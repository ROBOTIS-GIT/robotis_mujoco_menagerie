"""Plot paired offline EKF comparisons; no navigation success is inferred here."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def plot(root):
    aggregates=json.loads((root/'aggregates.json').read_text())
    series=json.loads((root/'series.json').read_text())
    scales=[.1,1.,10.]
    validation=[r for r in aggregates if r['split']=='validation']
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,2,figsize=(12,9),layout='constrained')
    for ax,metric,title,mult,fmt in [(axes[0,0],'position_rmse_m','Held-out position RMSE (cm)',100,'.1f'),
                                     (axes[0,1],'three_sigma_coverage','Held-out component coverage within +/-3 sigma (%)',100,'.1f')]:
        values=np.array([[next(r[metric] for r in validation if r['q_scale']==q and r['r_scale']==v)*mult for v in scales] for q in scales])
        shown=ax.imshow(values,cmap='viridis_r' if metric=='position_rmse_m' else 'viridis',aspect='auto')
        ax.set_xticks(range(3),['0.1','1','10']);ax.set_yticks(range(3),['0.1','1','10'])
        ax.set(xlabel='R covariance multiplier',ylabel='Q covariance multiplier',title=title)
        for i in range(3):
            for j in range(3):
                ax.text(j,i,format(values[i,j],fmt),ha='center',va='center',color='white' if np.dot(shown.cmap(shown.norm(values[i,j]))[:3],[.2126,.7152,.0722])<.5 else 'black',fontsize=11)
        fig.colorbar(shown,ax=ax,shrink=.8)
    for ax,name in [(axes[1,0],'nominal/seed-1'),(axes[1,1],'camera_noise_dropout/seed-1')]:
        trace=series[name];times=np.array(trace['time_s'])
        for variant,label,color in [('baseline','Q=1,R=1','#ba5b17'),('selected','Q=10,R=1 candidate','#027f95'),('no_camera','No camera corrections','#777777')]:
            error=np.array(trace[variant]['error'])
            ax.plot(times,np.linalg.norm(error[:,:2],axis=1),label=label,color=color,lw=1.5)
        ax.set(xlabel='Time on recorded trajectory (s)',ylabel='Position error (m)',title=name.replace('_',' '))
        ax.legend(fontsize=8)
    fig.suptitle('EKF Q/R sensitivity: fixed-trajectory replay, not navigation trials',fontsize=14)
    fig.savefig(root/'tuning-comparison.png',dpi=160)
    plt.close(fig)
    trace=series['camera_noise_dropout/seed-1'];times=np.array(trace['time_s'])
    fig,axes=plt.subplots(3,1,figsize=(11,9),layout='constrained',sharex=True)
    errors=np.array(trace['selected']['error']);cov=np.array(trace['selected']['covariance'])
    for i,ax in enumerate(axes):
        factor=180/np.pi if i==2 else 100
        sigma=3*np.sqrt(cov[:,i,i])*factor
        ax.fill_between(times,-sigma,sigma,color='#c3e7ec',label='Candidate +/-3 sigma')
        ax.plot(times,errors[:,i]*factor,color='#027f95',label='Candidate error')
        ax.plot(times,np.array(trace['baseline']['error'])[:,i]*factor,color='#ba5b17',lw=1,label='Baseline error')
        for t,visible in zip(times,trace['camera_visible']):
            if not visible: ax.axvspan(t,t+.1,color='#777777',alpha=.08,lw=0)
        ax.axhline(0,color='#999',lw=.5)
        ax.set(ylabel=['x error (cm)','y error (cm)','Heading error (deg)'][i])
        ax.legend(loc='upper right',fontsize=8)
    axes[-1].set_xlabel('Recorded time (s); grey bands mark missing camera observations')
    fig.suptitle('Held-out noisy-camera replay: Q x10, R x1\nConservative uncertainty; these are not independent Monte Carlo samples',fontsize=13)
    fig.savefig(root/'uncertainty-validation.png',dpi=160)
    plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    plot(p.parse_args().root.resolve())
