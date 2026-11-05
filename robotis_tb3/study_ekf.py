"""Paired offline Q/R sensitivity study. Recorded robot trajectories stay fixed.
Encoder increments in older logs are reconstructed from odometry only, never truth.
This is an estimator replay study, not a closed-loop navigation success study.
""" Okay,sohe's gota Read MefoPott D.<en-US> Where's thislocated?<en-US>It'slocatedinLQ off for somereason.<en-US>Is thismy ReadMe this looks likemy Read Me?<en-US> Yeah, okay,sohe didpull frommy thingit looks like and then continue off whatI wasdoingI just want to confirm that okayso which parts were you looking at that which parts were you looking at whichsections were you looking at?<en-US> No,no,asinasin the parts that we need to correct from hereon the part A project track A whichsections tellme tellme thefile namesholdon, letmesharescreen here tracka projectwhere isitRobotusTB threeare weusing these otherrobotics things by theway this FFWmaxmy op three we'reusing these other these rightso we can getrid of these let's just getrid of these for nowI don't believe they're beingused okaycoolsoI'mcurrentlyonmybranchI'mgoing to just continue offmybranch becauseI believeit's complete and justrequires the the video and thepipeline okayI deleted the otherroboticsfiles because weonly working with the Tp three turtle bot threeSimdo we want to justgo frommine andredo the video plus thepipeline then becauseI believemine was thefinalsetup if we're not touching what Ho's beendoing
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
from turtlebot_model import load_scene, step_pose_from_encoder, wrap_angle
from track_a_estimation import PoseEKF

INITIAL=np.array([.025,-.02,.025])
SCALES=(.1,1.,10.)


def encoder_increments(previous, current, spec):
    """Invert the differential-drive arc in logged encoder-only odometry."""
    turn=wrap_angle(current[2]-previous[2])
    mid=previous[2]+turn/2
    distance=float((current[:2]-previous[:2])@np.array([np.cos(mid),np.sin(mid)]))/np.sinc(turn/(2*np.pi))
    return np.array([(distance-spec.wheel_track_m*turn/2)/spec.wheel_radius_m,
                     (distance+spec.wheel_track_m*turn/2)/spec.wheel_radius_m])


def load_trace(path, spec):
    records=json.loads(path.read_text())
    previous=INITIAL.copy()
    increments=[]
    for record in records:
        current=np.asarray(record['odometry'])
        inc=np.asarray(record.get('encoder_increments',encoder_increments(previous,current,spec)))
        rebuilt=step_pose_from_encoder(previous,*inc,spec)
        np.testing.assert_allclose(rebuilt,current,atol=1e-10,rtol=0)
        increments.append(inc)
        previous=current
    return records,increments


def replay(records,increments,spec,q_scale=1.,r_scale=1.,remove_camera=False,keep_series=False):
    ekf=PoseEKF(INITIAL,spec,r_scale,q_scale)
    estimates=[];covariances=[];nis=[];accepted=[]
    for record,inc in zip(records,increments):
        ekf.predict(inc)
        observation=None if remove_camera else record['camera_measurement']
        used=ekf.correct(observation)
        if observation is not None:
            nis.append(ekf.last_nis)  # All attempted updates, including gated-out ones.
        accepted.append(used)
        estimates.append(ekf.pose.copy());covariances.append(ekf.P.copy())
    estimates=np.array(estimates);covariances=np.array(covariances)
    truth=np.array([r['truth'] for r in records])  # Evaluation only, after all filtering.
    error=estimates-truth;error[:,2]=[wrap_angle(x) for x in error[:,2]]
    nees=np.array([e@np.linalg.solve(P,e) for e,P in zip(error,covariances)])
    std=np.sqrt(np.diagonal(covariances,axis1=1,axis2=2))
    row={'q_scale':q_scale,'r_scale':r_scale,
         'position_rmse_m':float(np.sqrt(np.mean(np.sum(error[:,:2]**2,axis=1)))),
         'heading_rmse_deg':float(np.rad2deg(np.sqrt(np.mean(error[:,2]**2)))),
         'mean_nees':float(np.mean(nees)),
         'three_sigma_coverage':float(np.mean(np.abs(error)<=3*std)),
         'mean_attempted_nis':float(np.mean(nis)) if nis else None,
         'accepted_updates':ekf.accepted,'rejected_updates':ekf.rejected,
         'frames':len(records),'camera_attempts':len(nis)}
    series={'estimate':estimates.tolist(),'covariance':covariances.tolist(),'nees':nees.tolist(),
            'error':error.tolist(),'camera_accepted':accepted}
    return row,series if keep_series else None


def study(root,output):
    _,spec=load_scene()
    summaries=json.loads((root/'summary.json').read_text())
    data={}
    checks=[]
    for summary in summaries:
        name=f"{summary['condition']}/seed-{summary['seed']}"
        path=root/name/'trace.json'
        records,increments=load_trace(path,spec)
        data[name]=(summary,records,increments)
        _,series=replay(records,increments,spec,keep_series=True)
        expected=np.array([r['estimate'] for r in records])
        actual=np.array(series['estimate'])
        delta=actual-expected;delta[:,2]=[wrap_angle(x) for x in delta[:,2]]
        max_delta=float(np.max(np.abs(delta)))
        np.testing.assert_allclose(actual,expected,atol=1e-8,rtol=0)
        checks.append({'trace':name,'max_absolute_state_difference':max_delta,
                       'input_trace_sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    output.mkdir(parents=True,exist_ok=True)
    rows=[]
    for q in SCALES:
        for r in SCALES:
            for name,(summary,records,increments) in data.items():
                row,_=replay(records,increments,spec,q,r)
                row.update(trace=name,condition=summary['condition'],seed=summary['seed'],
                           split='development' if summary['seed']==0 and summary['condition'] in ('nominal','camera_noise_dropout') else 'validation')
                rows.append(row)
            print(f'Completed Q x{q:g}, R x{r:g}',flush=True)
    aggregates=[]
    for split in ('development','validation'):
        for q in SCALES:
            for r in SCALES:
                subset=[x for x in rows if x['split']==split and x['q_scale']==q and x['r_scale']==r]
                aggregates.append({'split':split,'q_scale':q,'r_scale':r,'trials':len(subset),
                   **{field:float(np.mean([x[field] for x in subset])) for field in
                      ('position_rmse_m','heading_rmse_deg','mean_nees','three_sigma_coverage','mean_attempted_nis','accepted_updates','rejected_updates')}})
    development=[a for a in aggregates if a['split']=='development']
    # Predefined pragmatic rule: among configurations with >=99% component-wise
    # 3-sigma coverage and mean NEES <=4 on development, minimise position RMSE.
    # These are engineering screening criteria, not a statistical consistency proof.
    eligible=[a for a in development if a['three_sigma_coverage']>=.99 and a['mean_nees']<=4]
    selected=min(eligible,key=lambda a:a['position_rmse_m']) if eligible else None
    selection={'rule':'Development only: >=99% 3-sigma component coverage and mean NEES <=4; then lowest mean position RMSE. No formal consistency claim.',
               'development_traces':['nominal/seed-0','camera_noise_dropout/seed-0'],
               'selected':selected,'baseline_equivalence_checks':checks,
               'caveat':'Fixed recorded trajectories; changing the estimator in a closed loop may change the path and measurements. Validation samples are few and temporally correlated.'}
    for name,obj in (('per-trial.json',rows),('aggregates.json',aggregates),('selection.json',selection)):
        (output/name).write_text(json.dumps(obj,indent=2)+'\n')
    with (output/'per-trial.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    variants={'baseline':(1.,1.),'low_R':(1.,.1),'high_R':(1.,10.),'low_Q':(.1,1.),'high_Q':(10.,1.)}
    if selected: variants['selected']=(selected['q_scale'],selected['r_scale'])
    series_output={}
    for name in ('nominal/seed-1','camera_noise_dropout/seed-1'):
        summary,records,increments=data[name]
        series_output[name]={'time_s':[r['time_s'] for r in records],
                            'truth':[r['truth'] for r in records],
                            'odometry':[r['odometry'] for r in records],
                            'camera_visible':[r['camera_measurement'] is not None for r in records]}
        for variant,(q,r) in variants.items():
            _,series=replay(records,increments,spec,q,r,keep_series=True)
            series_output[name][variant]=series
        ablation,series=replay(records,increments,spec,remove_camera=True,keep_series=True)
        series_output[name]['no_camera']=series
        series_output[name]['no_camera_metrics']=ablation
    (output/'series.json').write_text(json.dumps(series_output)+'\n')
    sources=output/'source-snapshot';sources.mkdir(exist_ok=True)
    for file in ('study_ekf.py','track_a_estimation.py','turtlebot_model.py'):
        shutil.copy2(Path(__file__).parent/file,sources/file)
    print(json.dumps(selection['selected']),flush=True)
    return selection


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args();study(args.root.resolve(),args.output.resolve())
