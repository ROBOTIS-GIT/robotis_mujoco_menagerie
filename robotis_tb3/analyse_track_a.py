"""Offline evaluation only. Requires matplotlib in addition to runtime dependencies."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def wrap(x):
    return (x+np.pi)%(2*np.pi)-np.pi


def analyse(root):
    results=json.loads((root/'summary.json').read_text())
    rows=[]
    for result in results:
        folder=root/result['condition']/f"seed-{result['seed']}"
        records=json.loads((folder/'trace.json').read_text())
        est=np.array([r['estimate'] for r in records])
        truth=np.array([r['truth'] for r in records])
        cov=np.array([r['covariance'] for r in records])
        error=est-truth
        error[:,2]=wrap(error[:,2])
        std=np.sqrt(np.diagonal(cov,axis1=1,axis2=2))
        row={k:result[k] for k in ('condition','seed','success','collision','simulation_seconds',
             'ekf_position_rmse_m','odometry_position_rmse_m','final_lateral_error_truth_m',
             'final_heading_error_truth_deg','mean_nees')}
        row['heading_rmse_deg']=float(np.rad2deg(np.sqrt(np.mean(error[:,2]**2))))
        row['three_sigma_component_coverage']=float(np.mean(np.abs(error)<=3*std))
        row['wheel_command_squared_integral']=float(.1*np.sum(np.array([r['wheel_commands'] for r in records])**2))
        cross_track=[]
        measurement_errors=[]
        for record in records:
            path=np.array(record['path'])
            if len(path):
                # Distance to discrete reference path points, not continuous trajectory error.
                cross_track.append(np.min(np.linalg.norm(path-np.array(record['truth'])[:2],axis=1)))
            if record['camera_measurement'] is not None:
                x,y,t=record['truth']
                dx,dy=3-x-.09*np.cos(t),-y-.09*np.sin(t)
                predicted=np.array([np.hypot(dx,dy),wrap(np.arctan2(dy,dx)-t),t])
                residual=np.array(record['camera_measurement'])-predicted
                residual[1:]=wrap(residual[1:])
                measurement_errors.append(residual)
        row['discrete_path_rmse_m']=float(np.sqrt(np.mean(np.square(cross_track)))) if cross_track else None
        if measurement_errors:
            rmse=np.sqrt(np.mean(np.array(measurement_errors)**2,axis=0))
            row.update(camera_range_rmse_m=float(rmse[0]),camera_bearing_rmse_deg=float(np.rad2deg(rmse[1])),
                       camera_heading_rmse_deg=float(np.rad2deg(rmse[2])))
        rows.append(row)
    with (root/'metrics.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        writer.writeheader();writer.writerows(rows)
    (root/'metrics.json').write_text(json.dumps(rows,indent=2)+'\n')
    groups={condition:[r for r in rows if r['condition']==condition] for condition in dict.fromkeys(r['condition'] for r in rows)}
    representative=json.loads((root/'nominal/seed-0/trace.json').read_text())
    est=np.array([r['estimate'] for r in representative]); truth=np.array([r['truth'] for r in representative])
    odo=np.array([r['odometry'] for r in representative]); cov=np.array([r['covariance'] for r in representative])
    times=np.array([r['time_s'] for r in representative])
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig=plt.figure(figsize=(13,11),layout='constrained')
    grid=fig.add_gridspec(3,2)
    ax=fig.add_subplot(grid[0,0])
    ax.plot(truth[:,0],truth[:,1],label='Truth',color='#1f2937',lw=2)
    ax.plot(est[:,0],est[:,1],label='EKF',color='#027f95',lw=1.5)
    ax.plot(odo[:,0],odo[:,1],label='Encoders only',color='#ba5b17',ls='--')
    for x,y,w,h in [(1.03,-.25,.44,.5),(1.9,-1.15,.4,.4)]:
        ax.add_patch(Rectangle((x,y),w,h,color='#b0b6ba',label=None))
    ax.scatter([2.36],[0],marker='*',s=140,color='#209060',label='Dock target')
    ax.set(xlabel='World x (m)',ylabel='World y (m)',title='Nominal route around obstacle',ylim=(-1.4,1.4))
    ax.axis('equal');ax.legend(fontsize=8)
    ax=fig.add_subplot(grid[0,1])
    names=list(groups);xx=np.arange(len(names))
    ekf=[np.mean([r['ekf_position_rmse_m'] for r in groups[n]]) for n in names]
    odom=[np.mean([r['odometry_position_rmse_m'] for r in groups[n]]) for n in names]
    ax.bar(xx-.18,ekf,.36,label='EKF',color='#027f95');ax.bar(xx+.18,odom,.36,label='Encoders only',color='#ba5b17')
    ax.set_xticks(xx,[n.replace('_','\n') for n in names],fontsize=8)
    ax.set(ylabel='Position RMSE (m)',title='Repeated-trial estimation error');ax.legend(fontsize=8)
    for i,n in enumerate(names):
        success=sum(r['success'] for r in groups[n]);total=len(groups[n])
        ax.text(i,max(ekf[i],odom[i])+.02,f'{success}/{total} success',ha='center',fontsize=8)
    error=est-truth;error[:,2]=wrap(error[:,2])
    for i in range(3):
        ax=fig.add_subplot(grid[1+i//2,i%2])
        sigma=3*np.sqrt(cov[:,i,i]);scale=180/np.pi if i==2 else 1
        ax.fill_between(times,-sigma*scale,sigma*scale,color='#bde6ec',label='+/-3 sigma')
        ax.plot(times,error[:,i]*scale,color='#027f95',label='EKF error')
        ax.axhline(0,color='#888',lw=.5)
        ax.set(xlabel='Time (s)',ylabel=('x error (m)','y error (m)','Heading error (deg)')[i],title=('Nominal x uncertainty','Nominal y uncertainty','Nominal heading uncertainty')[i])
        ax.legend(fontsize=8)
    ax=fig.add_subplot(grid[2,1]);ax.axis('off')
    text=['Evaluation scope',f"Trials: {len(rows)}; successes: {sum(r['success'] for r in rows)}",'Two seeds per condition; small sample.',
          'Truth used only for sensor simulation and evaluation.', 'Dock success also checks true range, lateral and yaw error.',
          'Coverage/NEES describe consistency, not a pass guarantee.', 'Camera residuals validate configured simulated intrinsics;',
          'they do not establish a real-camera calibration.']
    ax.text(0,1,'\n\n'.join(text),va='top',fontsize=10)
    fig.suptitle('Track A simulation evidence - estimated-state navigation',fontsize=16)
    fig.savefig(root/'evaluation.png',dpi=160)
    report=['TRACK A - LOCAL DEVELOPMENT EVALUATION','',f"Successes: {sum(r['success'] for r in rows)}/{len(rows)}",'']
    for name,group in groups.items():
        report.append(f"{name}: {sum(r['success'] for r in group)}/{len(group)} success; mean EKF position RMSE {np.mean([r['ekf_position_rmse_m'] for r in group]):.3f} m; encoder-only {np.mean([r['odometry_position_rmse_m'] for r in group]):.3f} m")
    report += ['', 'These are small simulation samples, not a claimed grade or completed submission.',
               'See metrics.csv for coverage, sensor residuals, path error and control effort.',
               'Each trial includes parameters, a trace and a source snapshot with SHA-256 hashes.',
               'Remaining: team details, calibration evidence, defended tuning comparisons, final project page and five-minute video.']
    (root/'Evaluation-summary.txt').write_text('\n'.join(report)+'\n')
    return rows

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    analyse(p.parse_args().root.resolve())
