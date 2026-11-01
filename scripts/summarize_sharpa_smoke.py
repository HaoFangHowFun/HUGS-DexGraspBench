"""Aggregate existing smoke outputs; never substitutes generated or simulated poses."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('run_dir',type=Path);parser.add_argument('--evaluation-replays',type=int,default=1);args=parser.parse_args()
    run=args.run_dir;bench=run/'bench/sharpa_smoke_right_full_sharpa_right'
    records=[]
    for path in sorted((bench/'evaluation').rglob('*.npy')):
        d=np.load(path,allow_pickle=True).item();diag=json.loads((bench/'diagnostics'/f'{path.stem}.json').read_text())
        hp=[p for p in diag['contact_pairs'].values() if 'object' in (p['body1'],p['body2']) and ('child-' in p['body1'] or 'child-' in p['body2'])]
        hh=[p for p in diag['contact_pairs'].values() if 'child-' in p['body1'] and 'child-' in p['body2']]
        final=np.array(diag['frames'][-1]['object_pose'])
        records.append(dict(candidate=int(path.stem.split('_')[0]),success=bool(d['succ_flag']),
                            evaluator_lift_error_m=float(d['delta_pos']),evaluator_angle_error_deg=float(d['delta_angle']),
                            actual_object_translation_m=float(np.linalg.norm(final[:3]-d['obj_pose'][:3])),
                            static_ho_penetration_m=float(d['ho_pene']),static_self_penetration_m=float(d['self_pene']),
                            static_contact_num=int(d['contact_num']),static_nearest_self_pair=d['self_signed_dist_body_pair'],
                            dynamic_ho_penetration_m=max([max(0.,-p['min_distance_m']) for p in hp],default=0.),
                            dynamic_self_penetration_m=max([max(0.,-p['min_distance_m']) for p in hh],default=0.),
                            dynamic_hand_object_contact=bool(hp),physics_steps=diag['steps'],warnings=sum(diag['warnings'])))
    solver=json.loads((run/'solver_summary.json').read_text());valid=json.loads((run/'candidate_validation.json').read_text())
    files=list((ROOT.parent/'HUGS-BODex/src/curobo/content/configs').rglob('*sharpa*.yml'))
    files += [ROOT.parent/'HUGS-BODex/src/curobo/content/configs/manip/sim_sharpa/tabletop_full.yml',ROOT/'config/task/eval.yaml',ROOT/'config/hand/sharpa_right.yaml',ROOT/'assets/hand/sharpa_wave/right.xml']
    report=dict(scope='Right hand, one DGN_2k bottle scene, 20 fixed-seed surface-initialized candidates',
                attempted=20,optimizer_completed=20,optimizer_success=solver['optimized_success'],converted=20,
                evaluated=len(records),physical_grasp_success=sum(r['success'] for r in records),
                valid_pregrasp_and_grasp=valid['valid_pregrasp_and_grasp'],all_three_stages_within_limits=valid['all_stages_within_limits'],
                pipeline_acceptance_passed=len(records)>0,full_grasp_evaluation_ready=False,large_scale_generation_ready=False,
                evaluator_semantics='Unchanged Shadow right_full tabletop evaluator; no thresholds relaxed.',
                evaluation_replays=args.evaluation_replays,
                simulation_warning_count=sum(r['warnings'] for r in records),
                dynamic_contact_cases=sum(r['dynamic_hand_object_contact'] for r in records),
                configuration_sha256={str(p.relative_to(ROOT.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
                candidates=sorted(records,key=lambda r:r['candidate']))
    for name in ['dynamic_ho_penetration_m','dynamic_self_penetration_m','static_self_penetration_m','actual_object_translation_m']:
        report['max_'+name]=max(r[name] for r in records)
    text=json.dumps(report,indent=2,default=lambda v:v.tolist() if isinstance(v,np.ndarray) else str(v))+'\n'
    (run/'summary.json').write_text(text);(ROOT/'docs/sharpa_phase2_validation.json').write_text(text)
    p=ROOT/'docs/sharpa_validation.json';combined=json.loads(p.read_text())
    combined['phase2']={k:v for k,v in report.items() if k not in ('candidates','configuration_sha256')}
    combined['phase2']['details']='sharpa_phase2_validation.json';p.write_text(json.dumps(combined,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('candidates','configuration_sha256')},indent=2))


if __name__=='__main__':main()
