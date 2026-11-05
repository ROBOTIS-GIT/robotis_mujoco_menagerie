"""Part D: LQR path tracker on a latched, straightened reference. No simulator imports.

Why a latched reference?
------------------------
The planner replans from the *estimated* position every frame, so a path that
always starts at the robot has near-zero cross-track error by construction.
ReferencePath instead keeps a world-frame polyline (A* route straightened by
line-of-sight on the occupancy grid) and only replaces it when it is blocked,
the goal moves, or the robot is lost. Error is then measured against a fixed line.

Error coordinates match track_a_control_design.py:
    e_y     = left-positive signed offset of the (estimated) robot from the segment
    e_theta = wrap(theta - psi_p)
"""
import numpy as np
from turtlebot_model import wrap_angle
from lqr.track_a_control_design import V_NOM, W_MAX, bryson_weights, lqr_discrete
from track_a_navigation import StateFeedbackTracker

SWITCH_DIST = 0.10      # advance to the next segment this close to the vertex [m]
ROTATE_ENTER = 0.40     # rad: rotate in place above this |e_theta| (same as baseline)
ROTATE_EXIT = 0.25      # rad: hysteresis, resume driving below this


def project_on_segment(a, b, p):
    """(s, length, psi_p, e_y) of point p relative to segment a->b.

    s is the unclipped along-track distance from a, e_y is left-positive."""
    d = b - a
    length = float(np.linalg.norm(d))
    if length < 1e-9:
        return 0.0, 0.0, 0.0, float(np.linalg.norm(p - a))
    t = d / length
    n = np.array([-t[1], t[0]])
    r = p - a
    return float(r @ t), length, float(np.arctan2(t[1], t[0])), float(r @ n)


class ReferencePath:
    def __init__(self, planner, blocked_frames=3, lost_distance=0.40):
        self.planner = planner
        self.blocked_frames = blocked_frames
        self.lost_distance = lost_distance
        self.points = []
        self.adoptions = 0
        self._blocked = 0

    # ---- occupancy helpers -------------------------------------------------
    def _cell_free(self, point):
        c = self.planner.cell(point)
        if not (0 <= c[0] < self.planner.shape[0] and 0 <= c[1] < self.planner.shape[1]):
            return False
        return not self.planner.occupied[c]

    def _segment_free(self, a, b, skip_near=None, skip_radius=0.15):
        n = max(2, int(np.ceil(np.linalg.norm(b - a) / 0.02)) + 1)
        for t in np.linspace(0.0, 1.0, n):
            q = a + (b - a) * t
            if skip_near is not None and np.linalg.norm(q - skip_near) < skip_radius:
                continue
            if not self._cell_free(q):
                return False
        return True

    def straighten(self, path):
        """Greedy line-of-sight shortcut of the grid path (string pulling)."""
        pts = [np.asarray(p, dtype=float) for p in path]
        out, i = [pts[0]], 0
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1 and not self._segment_free(pts[i], pts[j]):
                j -= 1
            out.append(pts[j])
            i = j
        return out

    # ---- latching logic ----------------------------------------------------
    def update(self, estimate_xy, path, goal):
        """Return the current reference polyline (a list; identity changes on adoption)."""
        if not path:
            return self.points
        need = not self.points
        if not need:
            if np.linalg.norm(self.points[-1] - goal) > 0.12:
                need = True
            elif self.distance_to(estimate_xy) > self.lost_distance:
                need = True
            else:
                blocked = any(not self._segment_free(a, b, skip_near=estimate_xy)
                              for a, b in zip(self.points[:-1], self.points[1:]))
                self._blocked = self._blocked + 1 if blocked else 0
                need = self._blocked >= self.blocked_frames
        if need:
            pts = self.straighten(path)
            pts[0] = np.asarray(estimate_xy, dtype=float).copy()   # start at the robot
            if np.linalg.norm(pts[-1] - goal) < 0.10:               # undo grid quantisation
                pts[-1] = np.asarray(goal, dtype=float).copy()
            if len(pts) >= 2 and np.linalg.norm(pts[1] - pts[0]) < 0.03:
                pts.pop(0 if len(pts) > 2 else 1)
            self.points = pts
            self.adoptions += 1
            self._blocked = 0
        return self.points

    # ---- error geometry (also used for ground-truth EVALUATION) -------------
    def distance_to(self, p):
        best = self._nearest(np.asarray(p, dtype=float))
        return np.inf if best is None else best[0]

    def project(self, pose_or_xy):
        """Nearest-segment (seg, s, e_y, e_theta). e_theta needs a 3-vector pose."""
        p = np.asarray(pose_or_xy, dtype=float)
        best = self._nearest(p)
        if best is None:
            return None, 0.0, np.inf, np.nan
        _, k, s, ey, psi = best
        e_th = wrap_angle(p[2] - psi) if p.size > 2 else np.nan
        return k, s, ey, e_th

    def _nearest(self, p):
        best = None
        for k, (a, b) in enumerate(zip(self.points[:-1], self.points[1:])):
            s, length, psi, ey = project_on_segment(a, b, p[:2])
            sc = min(max(s, 0.0), length)
            dist = np.hypot(sc - s, ey)
            if best is None or dist < best[0]:
                best = (dist, k, s, ey, psi)
        return best


class LQRPathTracker(StateFeedbackTracker):
    """State-feedback tracker: w = -K [e_y, e_theta], K from discrete LQR at v_nom.

    Uses the ESTIMATED pose only (the caller passes ekf.pose). Docking, LiDAR stop
    and arrival logic are inherited unchanged from the baseline tracker."""

    def __init__(self, r_scale=1.0, v=V_NOM, w_max=W_MAX, rotate_enter=ROTATE_ENTER, recovery=False):
        super().__init__(recovery=recovery)
        if not (0.1 <= rotate_enter <= 1.2):
            raise ValueError('rotate_enter must be between 0.1 and 1.2 rad')
        self.rotate_enter = rotate_enter
        self.rotate_exit = ROTATE_EXIT * rotate_enter / ROTATE_ENTER   # same hysteresis ratio
        Q, R = bryson_weights()
        self.K = lqr_discrete(v, Q, R * r_scale)[0]      # (K_ey, K_etheta)
        self.v, self.w_max = v, w_max
        self.reference = []
        self._ref_obj = None
        self.seg = 0
        self.rotating = False
        self.last_error = None       # (e_y, e_theta) the controller acted on

    def follow_path(self, estimate, path):
        pts = self.reference
        if pts is None or len(pts) < 2:
            return 0.0, 0.0, 'NO SAFE PATH - STOP'
        if pts is not self._ref_obj:                    # new reference adopted
            self._ref_obj, self.seg, self.rotating = pts, 0, False
        while self.seg < len(pts) - 2:                  # advance through vertices
            s, length, _, _ = project_on_segment(pts[self.seg], pts[self.seg + 1], estimate[:2])
            if s > length - SWITCH_DIST:
                self.seg += 1
            else:
                break
        _, _, psi, e_y = project_on_segment(pts[self.seg], pts[self.seg + 1], estimate[:2])
        e_th = wrap_angle(estimate[2] - psi)
        self.last_error = (e_y, e_th)
        if self.rotating and abs(e_th) < self.rotate_exit:
            self.rotating = False
        elif not self.rotating and abs(e_th) > self.rotate_enter:
            self.rotating = True
        if self.rotating:
            # v = 0: the error model is uncontrollable in e_y (rank drop), so turn on
            # heading error only instead of letting e_y drive a spin it cannot fix.
            speed, turn = 0.0, -self.K[1] * e_th
        else:
            speed, turn = self.v, -(self.K[0] * e_y + self.K[1] * e_th)
        turn = float(np.clip(turn, -self.w_max, self.w_max))
        return float(speed), turn, 'LIDAR PATH / LQR TRACK'
