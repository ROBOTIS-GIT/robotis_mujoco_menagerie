"""LiDAR occupancy planning and estimated-pose state feedback. No simulator imports."""
import heapq
import numpy as np
from turtlebot_model import wrap_angle
from track_a_estimation import LANDMARK, marker_measurement

GOAL = np.array([2.36, 0.0])


class OccupancyPlanner:
    resolution = .10
    lower = np.array([-.5, -1.4])
    shape = (41, 29)
    def __init__(self):
        self.occupied = np.zeros(self.shape, dtype=bool)
        self.path = []

    def cell(self, point):
        return tuple(np.rint((point-self.lower)/self.resolution).astype(int))

    def point(self, cell):
        return self.lower + np.asarray(cell)*self.resolution

    def update(self, pose, angles, ranges):
        self.occupied.fill(False)  # Local rolling map avoids accumulating pose drift.
        origin = pose[:2] - .049 * np.array([np.cos(pose[2]), np.sin(pose[2])])
        hits = origin + ranges[:,None]*np.column_stack((np.cos(angles+pose[2]), np.sin(angles+pose[2])))
        for hit, distance in zip(hits, ranges):
            if distance >= 3.9:
                continue
            cx, cy = self.cell(hit)
            for dx in range(-3,4):
                for dy in range(-3,4):
                    if dx*dx + dy*dy > 10:
                        continue
                    x,y = cx+dx,cy+dy
                    if 0<=x<self.shape[0] and 0<=y<self.shape[1]:
                        self.occupied[x,y] = True

    def plan(self, start, goal=GOAL):
        start, end = self.cell(start), self.cell(goal)
        frontier = [(0.0, start)]
        costs, parents = {start:0.0}, {}
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == end:
                path = [end]
                while path[-1] != start:
                    path.append(parents[path[-1]])
                self.path = [self.point(c) for c in reversed(path)]
                return self.path
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt = (current[0]+dx,current[1]+dy)
                if not (0<=nxt[0]<self.shape[0] and 0<=nxt[1]<self.shape[1]):
                    continue
                if self.occupied[nxt]:
                    continue
                if dx and dy and (self.occupied[current[0]+dx,current[1]] or
                                  self.occupied[current[0],current[1]+dy]):
                    continue
                # A small fixed upper-side preference prevents noisy symmetric
                # scans from flipping between equally short routes each frame.
                cost = costs[current] + np.hypot(dx,dy) + (1.0 if self.point(nxt)[1]<-.05 else 0)
                if cost >= costs.get(nxt, np.inf):
                    continue
                parents[nxt], costs[nxt] = current, cost
                priority = cost + np.hypot(nxt[0]-end[0],nxt[1]-end[1])
                heapq.heappush(frontier,(priority,nxt))
        self.path = []
        return []


RECOVERY_FRAMES = 10     # consecutive frames with no planner path before searching (1 s)
SEARCH_TURN = 0.8        # rad/s; the relay value, below it the wheels barely move (actuator dead zone)


class StateFeedbackTracker:
    def __init__(self, recovery=False):
        self.arrival_count = 0
        self.arrived = False
        self.docking = False
        self.position_captured = False
        self.missed_camera_frames = 0
        # Opt-in no-path recovery. Off by default so the baseline behaviour (and the
        # recorded NO SAFE PATH failures) are unchanged.
        self.recovery = recovery
        self.no_path_run = 0         # current run of consecutive frames without a planner path
        self.no_path_frames = 0      # total frames without a planner path (while not docking)
        self.search_frames = 0       # frames spent in the recovery search turn
        self._search_dir = 0.0

    def _search(self, estimate):
        """Rotate in place toward the ESTIMATED marker bearing to bring the marker into view.

        A stationary robot gets no new camera measurement, so a stale heading estimate (which
        tilts the LiDAR map and can close the route to the goal) can never be corrected.
        Turning lets the camera reacquire the marker; the EKF then fixes the heading."""
        bearing = marker_measurement(estimate)[1]
        if abs(bearing) > .10:
            self._search_dir = float(np.sign(bearing))
        elif self._search_dir == 0.0:
            self._search_dir = 1.0
        self.search_frames += 1
        return 0.0, self._search_dir * SEARCH_TURN, 'NO PATH - SEARCH FOR MARKER'

    @property
    def goal(self):
        return GOAL if self.docking else np.array([1.95, 0.0])

    def follow_path(self, estimate, path):
        """Baseline heuristic: pure-pursuit-style bearing law toward a point
        20 cm along the A* path. Subclasses (LQR, Part D) override this."""
        if not path:
            return 0.0,0.0,'NO SAFE PATH - STOP'
        target = path[-1]
        # Follow a point 20 cm along the collision-inflated grid path.
        for point in path:
            if np.linalg.norm(point-estimate[:2])>.20:
                target=point
                break
        delta=target-estimate[:2]
        bearing=wrap_angle(np.arctan2(delta[1],delta[0])-estimate[2])
        speed=.18*max(0.0,np.cos(bearing))
        turn=np.clip(2.8*bearing,-.8,.8)
        if abs(bearing)>.4:
            speed=0.0
        return float(speed),float(turn),'LIDAR PATH / EKF TRACK'

    def command(self, estimate, path, angles, ranges, camera_fresh):
        if self.arrived:
            return 0.0, 0.0, 'ARRIVED'
        if np.linalg.norm(estimate[:2]-np.array([1.95,0.0]))<.08:
            self.docking = True
        if self.docking:
            # Capture inside 3 cm, release outside 4 cm. An EKF correction can
            # move the estimate after stopping: resume positioning if needed.
            delta=GOAL-estimate[:2]
            distance=float(np.linalg.norm(delta))
            if distance<.03:
                self.position_captured=True
            elif distance>=.04:
                self.position_captured=False
            in_tolerance=(self.position_captured and distance<.04 and abs(estimate[2])<.08)
            if not in_tolerance:
                self.arrival_count=0
            if not camera_fresh:
                self.missed_camera_frames += 1
                # Allow gaps up to 0.2 s at the 10 Hz control rate. Missing
                # frames never add evidence or complete arrival by themselves.
                if self.missed_camera_frames>2:
                    self.arrival_count=0
                if in_tolerance:
                    return 0.0,0.0,'DOCK: HOLD - AWAIT CAMERA CONFIRMATION'
                # Translation stays disabled until the camera is reacquired.
                bearing=marker_measurement(estimate)[1]
                turn=float(np.clip(2.5*bearing,-.65,.65)) if abs(bearing)>.06 else 0.0
                return 0.0,turn,'DOCK: CAMERA REACQUIRE - NO TRANSLATION'
            self.missed_camera_frames=0
            if in_tolerance:
                self.arrival_count += 1
            if self.arrival_count>=5:
                self.arrived=True
                return 0.0,0.0,'ARRIVED'
            bearing=wrap_angle(-estimate[2]) if self.position_captured else wrap_angle(np.arctan2(delta[1],delta[0])-estimate[2])
            speed=0.0 if self.position_captured else max(.10,min(.18,.8*distance))
            turn=np.sign(bearing)*.8 if abs(bearing)>.06 else 0.0
            if abs(bearing)>.4:
                speed=0.0
            status='EKF VISUAL DOCK'
        else:
            self.no_path_run = 0 if path else self.no_path_run + 1
            self.no_path_frames += 0 if path else 1
            if self.recovery and self.no_path_run >= RECOVERY_FRAMES:
                speed,turn,status=self._search(estimate)
            else:
                speed,turn,status=self.follow_path(estimate,path)
        # Independent body-frame LiDAR stop envelope; angles are sensor readings.
        forward = ranges[np.abs(angles)<.40]
        if speed>0 and np.min(forward)<.36:
            return 0.0,0.0,'LIDAR SAFETY STOP'
        return float(speed),float(turn),status
