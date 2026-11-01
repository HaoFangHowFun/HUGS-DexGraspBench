"""SSH-friendly comparison of the actual BODex Shadow/Sharpa collision spheres.

Interactive local-frame sphere editing; explicit saves create separate drafts.
"""
import argparse
import copy
from datetime import datetime
from pathlib import Path
import queue
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh
import viser
import yaml

WORKSPACE = Path(__file__).resolve().parents[2]
BODEX = WORKSPACE / 'HUGS-BODex'
CONFIGS = BODEX / 'src/curobo/content/configs'
ASSETS = BODEX / 'src/curobo/content/assets'
sys.path.insert(0, str(WORKSPACE / 'HUGS-Sharpa-Assets/scripts'))
from validate_models import transform, urdf_fk


def load_hand(label, robot_name, suite, offset, yaw):
    kin = yaml.safe_load((CONFIGS / f'robot/{robot_name}').read_text())['robot_cfg']['kinematics']
    urdf = (ASSETS / kin['urdf_path']).resolve()
    tree = ET.parse(urdf).getroot()
    sphere_path = CONFIGS / 'robot' / kin['collision_spheres']
    sphere_data = yaml.safe_load(sphere_path.read_text())['collision_spheres']
    sphere_data = {name: sphere_data[name] for name in kin['collision_link_names']}
    manip = yaml.safe_load((CONFIGS / f'manip/{suite}/tabletop_full.yml').read_text())
    contact_names = manip['grasp_contact_strategy']['contact_points_name']
    contacts = set()
    for entry in contact_names:
        link, index = entry.rsplit('/', 1)
        index = int(index)
        assert link in sphere_data and 0 <= index < len(sphere_data[link]), entry
        contacts.add((link, index))
    meshes = {'Visual mesh': {}, 'URDF collision mesh': {}, 'Spheres': {}, 'Contact spheres': {}}
    for link in tree.findall('link'):
        name = link.get('name')
        for kind, tag in [('Visual mesh', 'visual'), ('URDF collision mesh', 'collision')]:
            parts = []
            for geometry in link.findall(tag):
                element = geometry.find('geometry/mesh')
                if element is None:
                    raise ValueError(f'Unsupported geometry: {label}/{name}')
                mesh = trimesh.load(urdf.parent / element.get('filename'), force='mesh')
                mesh.apply_scale(np.fromstring(element.get('scale', '1 1 1'), sep=' '))
                origin = geometry.find('origin')
                mesh.apply_transform(transform(**(origin.attrib if origin is not None else {})))
                parts.append(mesh)
            if parts:
                meshes[kind][name] = trimesh.util.concatenate(parts)
    template = trimesh.creation.icosphere(subdivisions=2, radius=1.)
    count = 0
    for name, balls in sphere_data.items():
        parts = {'Spheres': [], 'Contact spheres': []}
        for index, ball in enumerate(balls):
            radius = float(ball['radius'])
            center = np.array(ball['center'], dtype=float)
            assert radius > 0 and np.isfinite(center).all()
            mesh = template.copy()
            mesh.apply_scale(radius)
            mesh.apply_translation(center)
            kind = 'Contact spheres' if (name, index) in contacts else 'Spheres'
            parts[kind].append(mesh)
            count += 1
        for kind, group in parts.items():
            if group:
                meshes[kind][name] = trimesh.util.concatenate(group)
    # cuRobo builds joint order by traversing ee/link chains, not cspace order.
    # Shadow's runtime order starts with thumb although its cspace lists index first.
    by_child = {j.find('child').get('link'): j for j in tree.findall('joint')}
    names = []
    for tip in [kin['ee_link']] + kin['link_names']:
        chain = []
        while tip != kin['base_link']:
            joint = by_child[tip]
            chain.append(joint)
            tip = joint.find('parent').get('link')
        for joint in reversed(chain):
            name = joint.get('name')
            if joint.get('type') != 'fixed' and name not in names:
                names.append(name)
    assert set(names) == set(kin['cspace']['joint_names'])
    seed = manip['seeder_cfg']['q']
    assert len(seed) == len(names)
    zero = {j.get('name'): 0. for j in tree.findall('joint') if j.get('type') != 'fixed'}
    initial = {**zero, **dict(zip(names, seed))}
    layout = np.eye(4)
    layout[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
    layout[:3, 3] = offset
    poses = {key: urdf_fk(tree, values) for key, values in [('Joint zero', zero), ('BODex initial', initial)]}
    for fk in poses.values():
        for group in meshes.values():
            for name, mesh in group.items():
                assert name in fk and np.isfinite(mesh.vertices).all()
                assert np.max(mesh.extents) < 1., f'Unexpected units: {label}/{name}'
    print(f'{label}: {count} actual spheres; {len(sphere_data)} collision links; {sphere_path}', flush=True)
    print(f'{label}: {len(contacts)} designated contact spheres: {contact_names}', flush=True)
    return dict(label=label, count=count, contact_count=len(contacts), contact_names=contact_names,
                meshes=meshes, poses=poses, layout=layout, spheres=sphere_data,
                original_spheres=copy.deepcopy(sphere_data), contacts=contacts, sphere_path=sphere_path)


def sphere_group_mesh(hand, link, contact):
    parts = []
    for index, ball in enumerate(hand['spheres'][link]):
        if ((link, index) in hand['contacts']) != contact:
            continue
        mesh = trimesh.creation.icosphere(subdivisions=2, radius=float(ball['radius']))
        mesh.apply_translation(ball['center'])
        parts.append(mesh)
    return trimesh.util.concatenate(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8083)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--sharpa-spheres', choices=['34', '50-volume', '50-seed'], default='34',
                        help='Official 34 spheres, experimental volume fit, or its Shadow-derived seed.')
    args = parser.parse_args()
    # Align finger +Z and thumb +Y for comparison; preserve actual metric scale.
    sharpa_robot = {'34': 'right_sharpa_wave_sim.yml',
                    '50-volume': 'right_sharpa_wave_50_volume_preview.yml',
                    '50-seed': 'right_sharpa_wave_50_shadow_seed_preview.yml'}[args.sharpa_spheres]
    hands = [load_hand('Shadow', 'right_shadow_hand_sim.yml', 'sim_shadow', [0., -.17, 0.], 90.),
             load_hand('Sharpa', sharpa_robot, 'sim_sharpa', [0., .17, 0.], 0.)]
    if args.check:
        print('Both hands and both poses loaded; no geometry or collision settings changed.')
        return
    server = viser.ViserServer(host=args.host, port=args.port)
    server.scene.set_up_direction('+z')
    events = queue.SimpleQueue()
    try:
        with server.gui.add_folder('Collision sphere comparison'):
            pose = server.gui.add_dropdown('Joint pose', options=['Joint zero', 'BODex initial'], initial_value='Joint zero')
            mesh_type = server.gui.add_dropdown('Mesh', options=['Visual mesh', 'URDF collision mesh', 'Hidden'], initial_value='Visual mesh')
            show_balls = server.gui.add_checkbox('Show collision spheres', initial_value=True)
            contacts_only = server.gui.add_checkbox('Contact spheres only', initial_value=False)
            solid_contacts = server.gui.add_checkbox('Solid contact spheres', initial_value=True)
            wire = server.gui.add_checkbox('Sphere wireframe', initial_value=True)
            mesh_opacity = server.gui.add_slider('Mesh opacity', min=.05, max=1., step=.05, initial_value=.35)
            ball_opacity = server.gui.add_slider('Sphere opacity', min=.05, max=1., step=.05, initial_value=.8)
            server.gui.add_markdown(f'**Shadow: {hands[0]["count"]} spheres · Sharpa: {hands[1]["count"]} spheres**\n\n'
                                    f'**Green: designated contact spheres ({hands[0]["contact_count"]} / {hands[1]["contact_count"]}).** '
                                    '**Orange: other collision spheres.** '
                                    'Colors follow contact_points_name; other spheres on the same contact link may also use contact allowances.\n\n'
                                    f'Sharpa variant: **{args.sharpa_spheres}**. '
                                    'Actual BODex sphere radii; no enlargement. Same metric scale. '
                                    'Display roots are separated and aligned; no simulation. '
                                    'Overlapping spheres on the same link are normal. '
                                    'Self-collision margin buffers are not drawn. '
                                    'URDF collision meshes are not MuJoCo convex proxies.')
        controls = [pose, mesh_type, show_balls, contacts_only, solid_contacts, wire, mesh_opacity, ball_opacity]
        editable = hands[1]
        sphere_ids = editable['contact_names'] + [
            f'{link}/{i}' for link, balls in editable['spheres'].items() for i in range(len(balls))
            if (link, i) not in editable['contacts']]
        def get_ball(sphere_id, original=False):
            link, index = sphere_id.rsplit('/', 1)
            return editable['original_spheres' if original else 'spheres'][link][int(index)]
        first = get_ball(sphere_ids[0])
        with server.gui.add_folder('Edit Sharpa sphere'):
            sphere_select = server.gui.add_dropdown('Sphere (link/index)', options=sphere_ids, initial_value=sphere_ids[0])
            server.gui.add_markdown('**Units: mm. X/Y/Z are local to the selected link.** '
                                    'First five entries are contact targets. Changes appear immediately; '
                                    'Save draft writes a separate YAML, leaving source settings unchanged.')
            fields = {}
            for i, axis in enumerate(['X', 'Y', 'Z']):
                fields[axis] = server.gui.add_number(f'{axis} (mm)', initial_value=float(first['center'][i])*1000, step=.1)
            fields['R'] = server.gui.add_number('R (mm)', initial_value=float(first['radius'])*1000, min=.01, step=.1)
            reset_sphere = server.gui.add_button('Reset selected sphere')
            save_draft = server.gui.add_button('Save draft YAML')
            edit_status = server.gui.add_markdown('No edits saved.')
        def refresh_fields():
            ball = get_ball(sphere_select.value)
            for axis, value in zip(['X', 'Y', 'Z', 'R'], ball['center'] + [ball['radius']]):
                fields[axis].value = float(value)*1000
        @sphere_select.on_update
        def select_sphere(event):
            if event.client is not None:
                events.put(('select',))
        for axis, field in fields.items():
            def edit_field(event, axis=axis):
                if event.client is not None:
                    events.put(('edit', sphere_select.value, axis, float(event.target.value)))
            field.on_update(edit_field)
        reset_sphere.on_click(lambda event: events.put(('reset', sphere_select.value)))
        save_draft.on_click(lambda event: events.put(('save',)))
        filters = {}
        handles = []
        for hand in hands:
            label = hand['label']
            all_names = sorted(set().union(*(set(g) for g in hand['meshes'].values())))
            filters[label] = server.gui.add_dropdown(f'{label} link', options=['All'] + all_names, initial_value='All')
            controls.append(filters[label])
            server.scene.add_label(f'/{label}/label', f'{label}: {hand["count"]} spheres',
                                   position=hand['layout'][:3, 3] + [0., 0., .29])
            for kind, group in hand['meshes'].items():
                for name, mesh in group.items():
                    is_sphere = kind in ('Spheres', 'Contact spheres')
                    color = (40, 235, 100) if kind == 'Contact spheres' else (255, 155, 45) if is_sphere else (170, 185, 200)
                    handle = server.scene.add_mesh_simple(f'/{label}/{kind}/{name}',
                        mesh.vertices.astype(np.float32), mesh.faces.astype(np.uint32),
                        color=color, wireframe=kind == 'Spheres', opacity=.8 if is_sphere else .35,
                        side='double', visible=False)
                    handles.append((hand, kind, name, handle))
        local_axes = server.scene.add_frame('/Sharpa/selected_link_frame', axes_length=.025, axes_radius=.0005)
        selection = server.scene.add_icosphere('/Sharpa/selected_ball', radius=1., color=(255, 255, 80), wireframe=True)
        def update():
            for hand, kind, name, handle in handles:
                matrix = hand['layout'] @ hand['poses'][pose.value][name]
                handle.position = matrix[:3, 3]
                handle.wxyz = np.roll(Rotation.from_matrix(matrix[:3, :3]).as_quat(), 1)
                selected = filters[hand['label']].value
                is_sphere = kind in ('Spheres', 'Contact spheres')
                handle.visible = (selected == 'All' or selected == name) and (
                    (show_balls.value and (not contacts_only.value or kind == 'Contact spheres')) if is_sphere else mesh_type.value == kind)
                handle.opacity = ball_opacity.value if is_sphere else mesh_opacity.value
                if is_sphere:
                    handle.wireframe = wire.value and not (kind == 'Contact spheres' and solid_contacts.value)
            link, _ = sphere_select.value.rsplit('/', 1)
            frame = editable['layout'] @ editable['poses'][pose.value][link]
            local_axes.position = frame[:3, 3]
            local_axes.wxyz = np.roll(Rotation.from_matrix(frame[:3, :3]).as_quat(), 1)
            ball = get_ball(sphere_select.value)
            selection.position = (frame @ np.r_[ball['center'], 1.])[:3]
            selection.scale = float(ball['radius'])*1.03
            selection.visible = show_balls.value and filters['Sharpa'].value in ('All', link)
        for control in controls:
            control.on_update(lambda event: events.put(True))
        @server.on_client_connect
        def on_connect(client):
            client.camera.look_at = (0., 0., .13)
            client.camera.position = (.70, -.08, .30)
        update()
        print(f'Open http://localhost:{server.get_port()} after SSH forwarding. Ctrl+C stops.', flush=True)
        started = time.monotonic()
        while not args.duration or time.monotonic()-started < args.duration:
            if not events.empty():
                while not events.empty():
                    event = events.get()
                    if event is True:
                        continue
                    if event[0] == 'select':
                        refresh_fields()
                    elif event[0] in ('edit', 'reset'):
                        sphere_id = event[1]
                        ball = get_ball(sphere_id)
                        if event[0] == 'reset':
                            ball.update(copy.deepcopy(get_ball(sphere_id, original=True)))
                            refresh_fields()
                        else:
                            _, _, axis, value = event
                            if not np.isfinite(value) or (axis == 'R' and value <= 0):
                                refresh_fields()
                                continue
                            if axis == 'R':
                                ball['radius'] = value/1000
                            else:
                                ball['center'][['X', 'Y', 'Z'].index(axis)] = value/1000
                        link, index = sphere_id.rsplit('/', 1)
                        contact = (link, int(index)) in editable['contacts']
                        kind = 'Contact spheres' if contact else 'Spheres'
                        mesh = sphere_group_mesh(editable, link, contact)
                        for hand, group, name, handle in handles:
                            if hand is editable and group == kind and name == link:
                                handle.vertices = mesh.vertices.astype(np.float32)
                                handle.faces = mesh.faces.astype(np.uint32)
                        edit_status.content = 'Unsaved changes. Source YAML is unchanged.'
                    elif event[0] == 'save':
                        directory = WORKSPACE / 'benchmarks/sphere_edits'
                        directory.mkdir(parents=True, exist_ok=True)
                        stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                        path = directory / f'sharpa_{args.sharpa_spheres}_{stamp}.yml'
                        path.write_text(yaml.safe_dump({'collision_spheres': editable['spheres']}, sort_keys=False))
                        edit_status.content = f'Saved draft: `{path}`'
                        print(f'Saved sphere draft: {path}', flush=True)
                update()
            time.sleep(.04)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == '__main__':
    main()
