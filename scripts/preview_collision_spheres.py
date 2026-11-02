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


def load_hand(label, robot_name, suite, offset, yaw, manip_file=None, sphere_override=None):
    kin = yaml.safe_load((CONFIGS / f'robot/{robot_name}').read_text())['robot_cfg']['kinematics']
    urdf = (ASSETS / kin['urdf_path']).resolve()
    tree = ET.parse(urdf).getroot()
    sphere_path = Path(sphere_override) if sphere_override else CONFIGS / 'robot' / kin['collision_spheres']
    sphere_data = yaml.safe_load(sphere_path.read_text())['collision_spheres']
    sphere_data = {name: sphere_data[name] for name in kin['collision_link_names']}
    manip_path = CONFIGS / 'manip' / (manip_file or f'{suite}/tabletop_full.yml')
    manip = yaml.safe_load(manip_path.read_text())
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
    pose_joints = {'Joint zero': zero, 'BODex initial': initial}
    for title, mcp, pip, dip, spread in [('Mild flex', .35, .55, .4, 0.),
                                         ('Curled', .8, 1.2, .9, 0.),
                                         ('Spread + flex', .35, .55, .4, .2)]:
        values = dict(zero)
        for finger, short, direction in zip(['index', 'middle', 'ring', 'pinky'],
                                             ['FF', 'MF', 'RF', 'LF'], [-1., 0., .5, 1.]):
            if label == 'Sharpa':
                values.update({f'right_{finger}_MCP_FE': mcp, f'right_{finger}_MCP_AA': spread*direction,
                               f'right_{finger}_PIP': pip, f'right_{finger}_DIP': dip})
            else:
                values.update({f'rh_{short}J3': mcp, f'rh_{short}J4': spread*direction,
                               f'rh_{short}J2': pip, f'rh_{short}J1': pip})
        for joint in tree.findall('joint'):
            limit = joint.find('limit')
            if joint.get('name') in values and limit is not None:
                values[joint.get('name')] = float(np.clip(values[joint.get('name')],
                                                         float(limit.get('lower')), float(limit.get('upper'))))
        pose_joints[title] = values
    poses = {key: urdf_fk(tree, values) for key, values in pose_joints.items()}
    for fk in poses.values():
        for group in meshes.values():
            for name, mesh in group.items():
                assert name in fk and np.isfinite(mesh.vertices).all()
                assert np.max(mesh.extents) < 1., f'Unexpected units: {label}/{name}'
    print(f'{label}: {count} actual spheres; {len(sphere_data)} collision links; {sphere_path}', flush=True)
    print(f'{label}: {len(contacts)} designated contact spheres: {contact_names}', flush=True)
    print(f'{label}: manipulator configuration: {manip_path}', flush=True)
    return dict(label=label, count=count, contact_count=len(contacts), contact_names=contact_names,
                meshes=meshes, poses=poses, layout=layout, spheres=sphere_data,
                original_spheres=copy.deepcopy(sphere_data), contacts=contacts, sphere_path=sphere_path,
                tree=tree, pose_joints=pose_joints, joint_order=names)


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
    parser.add_argument('--sharpa-spheres', choices=['active', '34', '50-volume', '50-seed'], default='active',
                        help='Current retained Sharpa configuration, or an explicitly selected historical preset.')
    parser.add_argument('--sharpa-draft', type=Path,
                        help='Preview a separate sphere YAML with an original/draft comparison switch.')
    parser.add_argument('--sharpa-draft-label', default='Mesh-fit draft')
    parser.add_argument('--sharpa-reference-draft', type=Path,
                        help='Optional earlier draft for a third comparison option.')
    parser.add_argument('--sharpa-reference-label', default='Previous radius-only fit')
    parser.add_argument('--sharpa-pose-draft', type=Path,
                        help='Named-joint initial-pose draft; compare against Shadow retained initial pose.')
    parser.add_argument('--sharpa-reference-pose', type=Path,
                        help='Optional previous pose for comparison against Shadow.')
    args = parser.parse_args()
    # Align finger +Z and thumb +Y for comparison; preserve actual metric scale.
    active_manip = 'sim_sharpa/tabletop_full_50_restored.yml'
    sharpa_robot = {'active': yaml.safe_load((CONFIGS / 'manip' / active_manip).read_text())['robot_file'],
                    '34': 'right_sharpa_wave_sim.yml',
                    '50-volume': 'right_sharpa_wave_50_volume_preview.yml',
                    '50-seed': 'right_sharpa_wave_50_shadow_seed_preview.yml'}[args.sharpa_spheres]
    hands = [load_hand('Shadow', 'right_shadow_hand_sim.yml', 'sim_shadow', [0., -.17, 0.], 90.),
             load_hand('Sharpa', sharpa_robot, 'sim_sharpa', [0., .17, 0.], 0.,
                       manip_file=active_manip if args.sharpa_spheres == 'active' else None,
                       sphere_override=args.sharpa_draft)]
    initial_pose = 'BODex initial' if args.sharpa_spheres == 'active' else 'Joint zero'

    def add_pose_draft(path, title):
        draft_pose = yaml.safe_load(path.read_text())
        assert draft_pose['units'] == 'radians'
        values = draft_pose['joint_angles']
        assert set(values) == set(hands[1]['joint_order'])
        assert all(np.isfinite(float(value)) for value in values.values())
        for joint in hands[1]['tree'].findall('joint'):
            if joint.get('name') not in values:
                continue
            limit = joint.find('limit')
            assert limit is not None
            assert float(limit.get('lower')) <= values[joint.get('name')] <= float(limit.get('upper'))
        hands[1]['poses'][title] = urdf_fk(hands[1]['tree'], values)
        hands[1]['pose_joints'][title] = dict(values)
        hands[0]['poses'][title] = hands[0]['poses']['BODex initial']
        hands[0]['pose_joints'][title] = dict(hands[0]['pose_joints']['BODex initial'])
        print(f'Sharpa pose {title}: {path}; Shadow uses retained BODex initial.', flush=True)

    if args.sharpa_pose_draft:
        initial_pose = 'Draft initial vs Shadow'
        add_pose_draft(args.sharpa_pose_draft, initial_pose)
        if args.sharpa_reference_pose:
            add_pose_draft(args.sharpa_reference_pose, 'Previous thumb vs Shadow')
    variants = None
    if args.sharpa_draft:
        kin = yaml.safe_load((CONFIGS / 'robot' / sharpa_robot).read_text())['robot_cfg']['kinematics']
        original = yaml.safe_load((CONFIGS / 'robot' / kin['collision_spheres']).read_text())['collision_spheres']
        original = {link: original[link] for link in kin['collision_link_names']}
        assert all(len(original[link]) == len(balls) for link, balls in hands[1]['spheres'].items())
        variants = {'Original': original, args.sharpa_draft_label: copy.deepcopy(hands[1]['spheres'])}
        if args.sharpa_reference_draft:
            reference = yaml.safe_load(args.sharpa_reference_draft.read_text())['collision_spheres']
            assert all(len(reference[link]) == len(balls) for link, balls in original.items())
            variants[args.sharpa_reference_label] = {link: reference[link] for link in original}
        changed_contacts = sum(original[link][i] != hands[1]['spheres'][link][i]
                               for link, i in hands[1]['contacts'])
        variant_defaults = copy.deepcopy(variants)
        current_variant = args.sharpa_draft_label
    if args.check:
        print(f'Both hands and {len(hands[0]["poses"])} poses loaded; no geometry or collision settings changed.')
        return
    server = viser.ViserServer(host=args.host, port=args.port)
    server.scene.set_up_direction('+z')
    events = queue.SimpleQueue()
    try:
        with server.gui.add_folder('Collision sphere comparison'):
            pose = server.gui.add_dropdown('Joint pose', options=list(hands[0]['poses']), initial_value=initial_pose)
            if args.sharpa_pose_draft:
                server.gui.add_markdown('**Draft initial vs Shadow:** Sharpa uses the loaded initial-pose draft; '
                                        'Shadow uses its retained BODex initial pose. '
                                        '**Joint zero** shows zero angles on both hands. '
                                        '**BODex initial** shows both retained settings. '
                                        'Loading a pose draft here does not change the formal settings.')
            mesh_type = server.gui.add_dropdown('Mesh', options=['Visual mesh', 'URDF collision mesh', 'Hidden'],
                                               initial_value='URDF collision mesh' if variants and not changed_contacts else 'Visual mesh')
            if variants:
                variant_select = server.gui.add_dropdown('Sharpa spheres', options=list(variants), initial_value=current_variant)
                variant_select.on_update(lambda event: events.put(('variant', event.target.value)))
                contact_note = (f'{changed_contacts} green contact spheres were adjusted in this draft. '
                                if changed_contacts else 'Green contact targets are unchanged. ')
                server.gui.add_markdown('**Preview sphere draft.** '
                                        + contact_note + 'Switch Sharpa spheres to compare. '
                                        'Compare fit and finger coverage using the mesh, pose and finger controls. '
                                        'Formal grasp settings are unchanged.')
            show_balls = server.gui.add_checkbox('Show collision spheres', initial_value=True)
            contacts_only = server.gui.add_checkbox('Contact spheres only', initial_value=False)
            solid_contacts = server.gui.add_checkbox('Solid contact spheres', initial_value=True)
            wire = server.gui.add_checkbox('Sphere wireframe', initial_value=True)
            mesh_opacity = server.gui.add_slider('Mesh opacity', min=.05, max=1., step=.05, initial_value=.35)
            ball_opacity = server.gui.add_slider('Sphere opacity', min=.05, max=1., step=.05, initial_value=.8)
            server.gui.add_markdown(f'**Shadow: {hands[0]["count"]} spheres · Sharpa: {hands[1]["count"]} spheres**\n\n'
                                    f'**Green: designated contact spheres ({hands[0]["contact_count"]} / {hands[1]["contact_count"]}).** '
                                    '**Orange: other collision spheres.** '
                                    'All spheres participate in collision handling; green marks the grasp-attraction targets. '
                                    'Other spheres on contact-eligible links can have different clearance allowances.\n\n'
                                    f'Sharpa variant: **{args.sharpa_spheres}**. '
                                    'Sphere radii are drawn at their configured size. Same metric scale. '
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
        first_id = ('right_index_elastomer/0' if changed_contacts else 'right_index_MCP_VL/0') if variants else sphere_ids[0]
        first = get_ball(first_id)
        with server.gui.add_folder('Edit Sharpa sphere'):
            sphere_select = server.gui.add_dropdown('Sphere (link/index)', options=sphere_ids, initial_value=first_id)
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
        finger_prefixes = {
            'Sharpa': dict(zip(['Thumb', 'Index finger', 'Middle finger', 'Ring finger', 'Pinky'],
                               ['right_thumb_', 'right_index_', 'right_middle_', 'right_ring_', 'right_pinky_'])),
            'Shadow': dict(zip(['Thumb', 'Index finger', 'Middle finger', 'Ring finger', 'Pinky'],
                               ['rh_th', 'rh_ff', 'rh_mf', 'rh_rf', 'rh_lf']))}

        def link_visible(hand, name):
            selected = filters[hand['label']].value
            prefix = finger_prefixes[hand['label']].get(selected)
            return selected == 'All' or selected == name or (prefix is not None and name.startswith(prefix))

        for hand in hands:
            label = hand['label']
            all_names = sorted(set().union(*(set(g) for g in hand['meshes'].values())))
            filters[label] = server.gui.add_dropdown(f'{label} link',
                options=['All'] + list(finger_prefixes[label]) + all_names, initial_value='All')
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
                is_sphere = kind in ('Spheres', 'Contact spheres')
                handle.visible = link_visible(hand, name) and (
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
            selection.visible = show_balls.value and link_visible(editable, link)
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
                    if event[0] == 'variant':
                        variants[current_variant] = copy.deepcopy(editable['spheres'])
                        current_variant = event[1]
                        editable['spheres'] = copy.deepcopy(variants[current_variant])
                        editable['original_spheres'] = copy.deepcopy(variant_defaults[current_variant])
                        for hand, group, name, handle in handles:
                            if hand is editable and group in ('Spheres', 'Contact spheres'):
                                mesh = sphere_group_mesh(editable, name, group == 'Contact spheres')
                                handle.vertices = mesh.vertices.astype(np.float32)
                                handle.faces = mesh.faces.astype(np.uint32)
                        refresh_fields()
                        edit_status.content = f'Showing: {current_variant}. Formal settings are unchanged.'
                    elif event[0] == 'select':
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
