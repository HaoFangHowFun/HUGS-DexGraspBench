"""Browse single-scene Sharpa or Shadow smoke poses without stepping physics."""
from pathlib import Path
import queue
import sys
import time
import hydra
import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from task.eval_func.tabletop_mocap import tabletopMocapEval
from util.viewer_util import MjviserDebugViewer, normalize_debug_viewer_config


@hydra.main(config_path='../config', config_name='base', version_base=None)
def main(cfg):
    assert cfg.hand_name in ('sharpa_right', 'shadow')
    paths = sorted(Path(cfg.grasp_dir).rglob('*_grasp.npy'), key=lambda p: int(p.stem.split('_')[0]))
    if not paths:
        raise ValueError(f'No poses in {cfg.grasp_dir}')
    runner = tabletopMocapEval(str(paths[0]), cfg)
    ho = runner.mj_ho
    records = [np.load(p, allow_pickle=True).item() for p in paths]
    stages = ['pregrasp', 'grasp', 'squeeze']
    stage = stages.index(str(cfg.get('preview_stage', 'pregrasp')))
    index = int(cfg.get('preview_index', 0)) % len(records)
    for record in records:
        assert record['obj_path'] == records[0]['obj_path']
        np.testing.assert_allclose(record['obj_pose'], records[0]['obj_pose'])
        assert record['obj_scale'] == records[0]['obj_scale']
        order = [record['joint_names'].index(n) for n in ho.get_joint_names()[1:]]
        for name in stages:
            q = record[name + '_qpos'].copy()
            q[7:] = q[7:][order]
            record[name + '_qpos'] = q
    contact_viewer = None
    contact_handles = []
    contact_info = None
    def show():
        record = records[index]
        ho.reset_pose_qpos(record[stages[stage] + '_qpos'], record['obj_pose'], set_ctrl=False)
        mujoco.mj_forward(ho.model, ho.data)
        if contact_viewer is not None:
            for handle in contact_handles:
                handle.remove()
            contact_handles.clear()
            contacts, _ = ho.get_contact_info(record[stages[stage] + '_qpos'], record['obj_pose'], obj_margin=.002)
            colors = {'thumb': (235,80,80), 'index': (255,165,35), 'middle': (60,200,90),
                      'ring': (65,140,255), 'pinky': (190,90,235), 'other': (200,200,200)}
            aliases = {'rh_th':'thumb', 'rh_ff':'index', 'rh_mf':'middle', 'rh_rf':'ring', 'rh_lf':'pinky'}
            rows = []
            for i, contact in enumerate(contacts):
                if contact['contact_dist'] > .002:
                    continue
                body = contact['body1_name']
                finger = next((v for k,v in aliases.items() if body.startswith(k)), 'other')
                if finger == 'other':
                    finger = next((f for f in colors if f != 'other' and f in body), 'other')
                point = np.asarray(contact['contact_pos']).copy()
                normal = np.asarray(contact['contact_normal']).copy()
                color = colors[finger]
                near = contact['contact_dist'] > 0
                contact_handles.append(contact_viewer.server.scene.add_icosphere(
                    f'/contacts/{i}', radius=.0015, color=color, position=point, wireframe=near))
                contact_handles.append(contact_viewer.server.scene.add_line_segments(
                    f'/contact_normals/{i}', points=np.array([[point,point+.008*normal]],dtype=np.float32),
                    colors=np.array([[color,color]],dtype=np.uint8), line_width=2.))
                rows.append(f"{finger} · {body}: {float(contact['contact_dist'])*1000:.2f} mm")
            contact_info.content = ('**Generated pose contacts (no physics stepping)**\n\n'
                'Solid: distance ≤ 0. Wireframe: positive gap ≤ 2 mm. Lines: hand→object normal.\n\n'
                'Thumb red · index orange · middle green · ring blue · pinky purple.\n\n'
                + '\n\n'.join(rows or ['No contacts within 2 mm.']))
        print(f'{paths[index].stem}: {stages[stage]}', flush=True)
    show()
    if cfg.get('preview_check', False):
        for index in range(len(records)):
            for stage in range(3):
                show()
        return
    if cfg.get('preview_backend', 'mjviser') == 'mjviser':
        viewer = MjviserDebugViewer(ho.model, ho.data, normalize_debug_viewer_config(cfg.task.viewer), 0.)
        if cfg.get('preview_contacts', False):
            contact_viewer = viewer
            with viewer.server.gui.add_folder('Contact points'):
                contact_info = viewer.server.gui.add_markdown('')
            show()
            viewer.sync()
        changes = queue.SimpleQueue()
        try:
            with viewer.server.gui.add_folder('Grasp pose preview'):
                candidate = viewer.server.gui.add_slider('Candidate', min=0, max=len(records)-1, step=1, initial_value=index)
                pose_stage = viewer.server.gui.add_dropdown('Pose', options=stages, initial_value=stages[stage])
                previous = viewer.server.gui.add_button('Previous candidate')
                following = viewer.server.gui.add_button('Next candidate')
                viewer.server.gui.add_markdown(f'{cfg.hand_name}: static poses only; no physics stepping.')
            @candidate.on_update
            def on_candidate(event):
                changes.put(('candidate', int(event.target.value)))
            @pose_stage.on_update
            def on_stage(event):
                changes.put(('stage', stages.index(event.target.value)))
            @previous.on_click
            def on_previous(event):
                candidate.value = (int(candidate.value)-1) % len(records)
            @following.on_click
            def on_next(event):
                candidate.value = (int(candidate.value)+1) % len(records)
            @viewer.server.on_client_connect
            def on_connect(client):
                center = (ho.get_obj_pose()[:3] + ho.data.qpos[:3]) / 2
                client.camera.look_at = center
                client.camera.position = center + np.array([.3, -.3, .2])
            print('Use the browser Grasp pose preview controls. Ctrl+C stops the server.', flush=True)
            started = time.monotonic()
            while True:
                changed = False
                while not changes.empty():
                    kind, value = changes.get()
                    if kind == 'candidate':
                        index = value
                    else:
                        stage = value
                    changed = True
                if changed:
                    show()
                    viewer.sync()
                if cfg.get('preview_duration', 0) and time.monotonic()-started > cfg.preview_duration:
                    break
                time.sleep(.02)
        except KeyboardInterrupt:
            pass
        finally:
            viewer.close()
        return
    keys = queue.SimpleQueue()
    print('N/P: next/previous candidate; 1/2/3: pregrasp/grasp/squeeze. Static poses; no physics steps.')
    with mujoco.viewer.launch_passive(ho.model, ho.data, key_callback=keys.put) as viewer:
        viewer.cam.lookat[:] = (ho.get_obj_pose()[:3] + ho.data.qpos[:3]) / 2
        viewer.cam.distance = .45
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        while viewer.is_running():
            while not keys.empty():
                key = keys.get()
                if key in (78, 80):
                    index = (index + (1 if key == 78 else -1)) % len(records)
                elif key in (49, 50, 51):
                    stage = key - 49
                else:
                    continue
                with viewer.lock():
                    show()
            viewer.sync()
            time.sleep(.02)


if __name__ == '__main__':
    main()
