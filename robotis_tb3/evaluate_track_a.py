"""Reproducible repeated trials; failures remain in the output summary."""
import json
from pathlib import Path
from track_a_project import parser,run

if __name__=='__main__':
    p=parser()
    p.add_argument('--trials',type=int,default=3,help='Seeds per condition')
    args=p.parse_args()
    root=Path(args.output).resolve()
    root.mkdir(parents=True,exist_ok=True)
    conditions={
        'nominal':{},
        'camera_noise_dropout':{'pixel_noise':5,'dropout':.30},
        'shifted_obstacle':{'obstacle_shift':.15,'encoder_bias':.004},
        'slippery_floor':{'friction':.6},
    }
    results=[]
    for condition,changes in conditions.items():
        for seed in range(args.trials):
            trial=parser().parse_args([])
            trial.headless=True
            trial.seconds=args.seconds
            trial.seed=seed
            trial.output=str(root/condition/f'seed-{seed}')
            for name,value in changes.items():
                setattr(trial,name,value)
            result=run(trial)
            result['condition']=condition
            results.append(result)
            (root/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
    print(f"Successes {sum(r['success'] for r in results)}/{len(results)}")
