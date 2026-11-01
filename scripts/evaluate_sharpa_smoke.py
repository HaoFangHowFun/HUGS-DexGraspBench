"""Run the unchanged tabletop evaluator with read-only per-step diagnostics.

Restricted to Sharpa right and Shadow control; no evaluator parameters changed.
"""
import json
from pathlib import Path
import sys

import hydra
import mujoco
import numpy as np
from omegaconf import OmegaConf

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from task.eval_func.tabletop_mocap import tabletopMocapEval


@hydra.main(config_path='../config',config_name='base',version_base=None)
def main(cfg):
    assert cfg.hand_name in ('sharpa_right', 'shadow') and cfg.setting=='tabletop'
    output=Path(cfg.save_dir)/'diagnostics';output.mkdir(parents=True,exist_ok=True)
    OmegaConf.save(cfg,output/'resolved_config.yaml',resolve=True)
    files=sorted(Path(cfg.grasp_dir).rglob('*.npy'))
    files=files[max(0,cfg.task.start):None if cfg.task.end<0 else cfg.task.end]
    summaries=[]
    original=mujoco.mj_step
    for path in files:
        runner=tabletopMocapEval(str(path),cfg)
        ho=runner.mj_ho
        rows=[];pairs={};warning_max=np.zeros(len(ho.data.warning),dtype=int)
        steps=0
        def observe(model,data):
            nonlocal steps
            steps+=1
            warning_max[:]=np.maximum(warning_max,np.array([w.number for w in data.warning]))
            contacts=[]
            for c in data.contact:
                ids=[int(c.geom1),int(c.geom2)]
                bodies=[model.body(int(model.geom_bodyid[g])).name for g in ids]
                a,b=[model.geom(g).name or f'{body}:geom{g}' for g,body in zip(ids,bodies)]
                key=f'{a} / {b}'
                entry=pairs.setdefault(key,dict(min_distance_m=0.,observations=0,body1=bodies[0],body2=bodies[1]))
                entry['min_distance_m']=min(entry['min_distance_m'],float(c.dist));entry['observations']+=1
                if steps%10==0:
                    contacts.append(dict(geom1=a,geom2=b,body1=bodies[0],body2=bodies[1],distance_m=float(c.dist),point_m=c.pos.tolist(),normal=c.frame[:3].tolist()))
            if steps%10==0:
                rows.append(dict(time_s=float(data.time),object_pose=ho.get_obj_pose().tolist(),contacts=contacts))
        def step(model,data,*args,**kwargs):
            result=original(model,data,*args,**kwargs)
            if model is ho.model:observe(model,data)
            return result
        mujoco.mj_step=step
        try:
            runner.run()
        finally:
            mujoco.mj_step=original
        record=dict(input=str(path),steps=steps,warnings=warning_max.tolist(),contact_pairs=pairs,frames=rows,
                    sampling='Contact minima and warnings every physics step; points and object pose every 10 steps.',
                    phase1_limitations=('Convex artifacts remain possible: middle_DP/ring_MP and ring_DP/pinky_MP at large MCP_AA.' if cfg.hand_name=='sharpa_right' else 'Shadow baseline geometry unchanged.'))
        (output/(path.stem+'.json')).write_text(json.dumps(record,indent=2)+'\n')
        summaries.append({k:v for k,v in record.items() if k not in ('frames','contact_pairs')})
        print(path.name,'steps',steps,'warnings',int(warning_max.sum()),flush=True)
    (output/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')


if __name__=='__main__':main()
