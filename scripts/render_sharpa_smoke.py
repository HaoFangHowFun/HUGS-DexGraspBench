"""Render generated grasp poses using the exact evaluation model, without stepping."""
from pathlib import Path
import sys
import hydra
import mujoco
import numpy as np
from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from task.eval_func.tabletop_mocap import tabletopMocapEval


@hydra.main(config_path='../config',config_name='base',version_base=None)
def main(cfg):
    assert cfg.hand_name in ('sharpa_right', 'shadow')
    sheet=Image.new('RGB',(1280,1000),'white')
    for panel,index in enumerate([0,5,15,18]):
        path=next(Path(cfg.grasp_dir).rglob(f'{index}_grasp.npy'))
        runner=tabletopMocapEval(str(path),cfg);ho=runner.mj_ho
        ho.reset_pose_qpos(runner.grasp_data['grasp_qpos'],runner.grasp_data['obj_pose'],set_ctrl=False)
        mujoco.mj_forward(ho.model,ho.data)
        camera=mujoco.MjvCamera();camera.lookat[:]=(ho.get_obj_pose()[:3]+ho.data.qpos[:3])/2
        camera.distance=.40;camera.azimuth=135;camera.elevation=-20
        with mujoco.Renderer(ho.model,height=460,width=640) as renderer:
            renderer.update_scene(ho.data,camera);pixels=renderer.render()
        x=(panel%2)*640;y=(panel//2)*500
        sheet.paste(Image.fromarray(pixels),(x,y+35))
        ImageDraw.Draw(sheet).text((x+15,y+10),f'Candidate {index}: MuJoCo grasp pose (before dynamics)',fill='black')
    output=Path(cfg.save_dir)/'diagnostics/candidate_mujoco.png';output.parent.mkdir(parents=True,exist_ok=True);sheet.save(output)


if __name__=='__main__':main()
