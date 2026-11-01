"""Deterministic motion and object-contact checks using the unchanged Bench MjHO."""
from pathlib import Path
import sys

import mujoco
import numpy as np
import trimesh

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))
from util.hand_util import MjHO

ROOT_POSE = np.array([0, 0, 0.5, 1, 0, 0, 0])
FAR_OBJECT = np.array([1, 1, 1, 1, 0, 0, 0])
FINGERS = ("thumb", "index", "middle", "ring", "pinky")


def make_scene(directory, xml, shape="box"):
    directory = Path(directory) / shape
    folder = directory / "urdf/meshes"
    folder.mkdir(parents=True, exist_ok=True)
    meshes = {
        "box": lambda: trimesh.creation.box(extents=[0.016]*3),
        "sphere": lambda: trimesh.creation.icosphere(subdivisions=2, radius=0.008),
        "cylinder": lambda: trimesh.creation.cylinder(radius=0.008, height=0.016, sections=32),
    }
    meshes[shape]().export(folder / "convex_piece_0.obj")
    scene = MjHO(str(directory), 1.0, 500.0, None, str(xml), True, None, [0.5, 0.01], False)
    mujoco.mj_forward(scene.model, scene.data)
    scene._init_after_first_fk()
    return scene


def contacts(scene):
    result = []
    for index, contact in enumerate(scene.data.contact):
        force = np.zeros(6)
        mujoco.mj_contactForce(scene.model, scene.data, index, force)
        result.append(dict(
            body1=scene.model.body(scene.model.geom_bodyid[contact.geom1]).name,
            body2=scene.model.body(scene.model.geom_bodyid[contact.geom2]).name,
            point_m=contact.pos.tolist(), normal=contact.frame[:3].tolist(),
            distance_m=float(contact.dist), normal_force_n=float(force[0]),
        ))
    return result


def reset(scene, q=None, object_pose=FAR_OBJECT):
    q = np.zeros(22) if q is None else q
    scene.reset_pose_qpos(np.r_[ROOT_POSE, q], object_pose)
    scene.set_ext_force_on_obj(np.zeros(6))


def names_and_limits(scene):
    return scene.get_joint_names()[1:], scene.model.jnt_range[1:23].copy()


def target_vector(scene, values):
    names, _ = names_and_limits(scene)
    return np.array([values.get(name.split("_", 1)[1], 0.0) for name in names])


def representative_poses(scene):
    poses = {"canonical_zero": {}}
    for finger in FINGERS:
        if finger == "thumb":
            poses["thumb_opposition"] = dict(thumb_CMC_FE=.9, thumb_CMC_AA=.2,
                                              thumb_MCP_FE=.6, thumb_MCP_AA=.1, thumb_IP=.8)
        else:
            poses[f"{finger}_flexion"] = {f"{finger}_MCP_FE": 1.1, f"{finger}_PIP": 1.3,
                                           f"{finger}_DIP": 1.0}
    poses["extension"] = {**{f"{finger}_MCP_FE": -.16 for finger in FINGERS[1:]},
                            "thumb_CMC_FE": -.05, "thumb_MCP_FE": -.45}
    poses["abduction"] = {f"{finger}_MCP_AA": angle for finger, angle in
                           zip(FINGERS[1:], [-.25, -.08, .08, .25])}
    poses["combined_closing"] = {key: value for name, values in poses.items()
                                  if name.endswith("_flexion") for key, value in values.items()}
    poses["combined_closing"].update(poses["thumb_opposition"], pinky_CMC=.2)
    poses["pinky_cupping"] = {"pinky_CMC": .25}
    targets = {name: target_vector(scene, values) for name, values in poses.items()}
    # Deeper closure is deliberately tested separately as a self-contact probe.
    targets["combined_closing"] *= .5
    return targets


def run_motion(scene, target, ramp_steps=500, hold_steps=250, return_to_zero=True):
    reset(scene)
    _, limits = names_and_limits(scene)
    peak_error = peak_limit = peak_effort = peak_speed = 0.0
    worst_contacts = []
    worst_depth = 0.0
    pair_depths = {}
    snapshots = []
    for step in range(ramp_steps + hold_steps + (ramp_steps + hold_steps if return_to_zero else 0)):
        if step < ramp_steps + hold_steps:
            fraction = min((step+1)/ramp_steps, 1)
            command = target * (0.5-0.5*np.cos(np.pi*fraction))
        else:
            fraction = min((step-ramp_steps-hold_steps+1)/ramp_steps, 1)
            command = target * (0.5+0.5*np.cos(np.pi*fraction))
        scene.data.ctrl[:] = command
        mujoco.mj_step(scene.model, scene.data)
        error = float(np.max(np.abs(scene.data.qpos[7:29]-command)))
        peak_error = max(peak_error, error)
        peak_limit = max(peak_limit, float(np.max(np.maximum(limits[:, 0]-scene.data.qpos[7:29],
                                                            scene.data.qpos[7:29]-limits[:, 1]))))
        peak_effort = max(peak_effort, float(np.max(np.abs(scene.data.actuator_force))))
        peak_speed = max(peak_speed, float(np.max(np.abs(scene.data.qvel[6:28]))))
        for contact in scene.data.contact:
            first = scene.model.body(scene.model.geom_bodyid[contact.geom1]).name
            second = scene.model.body(scene.model.geom_bodyid[contact.geom2]).name
            key = first + "/" + second
            pair_depths[key] = max(pair_depths.get(key, 0.0), max(0.0, -float(contact.dist)))
        depth = max([0.0] + [-float(c.dist) for c in scene.data.contact])
        if depth > worst_depth:
            worst_depth = depth
            worst_contacts = contacts(scene)
        if step in (ramp_steps-1, ramp_steps+hold_steps-1):
            snapshots.append(dict(step=step+1, error_rad=error, actual_joint_qpos=scene.data.qpos[7:29].tolist()))
        if not np.isfinite(scene.data.qpos).all() or not np.isfinite(scene.data.qvel).all():
            raise AssertionError("Non-finite state during motion")
    return dict(target_qpos=target.tolist(), ramp_s=ramp_steps*scene.model.opt.timestep,
                hold_s=hold_steps*scene.model.opt.timestep, returned_to_zero=return_to_zero,
                peak_tracking_error_rad=peak_error, settled_tracking_error_rad=snapshots[-1]["error_rad"],
                return_error_rad=float(np.max(np.abs(scene.data.qpos[7:29]))) if return_to_zero else None,
                peak_limit_violation_rad=peak_limit, peak_actuator_force_nm=peak_effort,
                peak_joint_speed_rad_s=peak_speed, self_contact_pairs=pair_depths,
                worst_contacts=worst_contacts, snapshots=snapshots,
                warnings=scene.data.warning.number.tolist())


def static_range_scan(scene):
    """Sample each independent joint through its full URDF range, without claiming feasibility."""
    names, limits = names_and_limits(scene)
    pairs = {}
    palm_samples = []
    samples = 0
    for index, name in enumerate(names):
        for angle in np.linspace(*limits[index], 9):
            q = np.zeros(22)
            q[index] = angle
            reset(scene, q)
            samples += 1
            sampled_pairs = set()
            for contact in contacts(scene):
                key = contact["body1"] + "/" + contact["body2"]
                if "hand_C_MC" in key and key not in sampled_pairs:
                    palm_samples.append(dict(joint=name, angle_rad=float(angle), qpos=q.tolist(), contact=contact))
                    sampled_pairs.add(key)
                if key not in pairs or contact["distance_m"] < pairs[key]["contact"]["distance_m"]:
                    pairs[key] = dict(joint=name, angle_rad=float(angle), qpos=q.tolist(), contact=contact)
    return dict(samples=samples, contact_pairs=pairs, palm_contact_samples=palm_samples)


def limit_checks(scene):
    """Drive both PIP limits with an out-of-range command; test actual ctrl clipping and soft limits."""
    names, limits = names_and_limits(scene)
    index = next(i for i, name in enumerate(names) if name.endswith("index_PIP"))
    results = []
    for label, bound in zip(("lower", "upper"), limits[index]):
        target = np.zeros(22)
        target[index] = bound
        result = run_motion(scene, target, return_to_zero=False)
        scene.data.ctrl[index] = bound + (-0.2 if label == "lower" else 0.2)
        max_violation = 0.0
        for _ in range(250):
            mujoco.mj_step(scene.model, scene.data)
            q = scene.data.qpos[7+index]
            max_violation = max(max_violation, limits[index, 0]-q, q-limits[index, 1])
        result.update(joint=names[index], boundary=label, out_of_range_ctrl=float(scene.data.ctrl[index]),
                      final_qpos=float(scene.data.qpos[7+index]), clipping_limit_violation_rad=float(max_violation),
                      warnings=scene.data.warning.number.tolist())
        results.append(result)
    return results


def find_object_placement(scene, side, finger):
    """Find a shallow isolated pad contact; record the chosen direction and pose."""
    reset(scene)
    body_name = f"child-{side}_{finger}_elastomer"
    body_id = scene.model.body(body_name).id
    geom_id = next(i for i in range(scene.model.ngeom)
                   if scene.model.geom_bodyid[i] == body_id and scene.model.geom_contype[i])
    mesh = scene.model.mesh(scene.model.geom_dataid[geom_id])
    vertices = scene.model.mesh_vert[mesh.vertadr[0]:mesh.vertadr[0]+mesh.vertnum[0]]
    vertices = vertices @ scene.data.geom_xmat[geom_id].reshape(3, 3).T + scene.data.geom_xpos[geom_id]
    for direction in ([1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1]):
        direction = np.array(direction, dtype=float)
        support = vertices @ direction
        point = vertices[support > support.max()-0.0001].mean(axis=0)
        for overlap in (.0002, .0005, .001):
            pose = np.r_[point + direction*(.008-overlap), 1, 0, 0, 0]
            reset(scene, object_pose=pose)
            hits = contacts(scene)
            object_hits = [c for c in hits if "object" in (c["body1"], c["body2"])]
            if object_hits and all(body_name in (c["body1"], c["body2"]) for c in object_hits):
                normal = np.mean([np.array(c["normal"])*(1 if c["body2"] == "object" else -1)
                                  for c in object_hits], axis=0)
                normal /= np.linalg.norm(normal)
                return pose, normal, object_hits
    raise AssertionError(f"Cannot place object at {body_name}")


def object_checks(scene, side, shape):
    results = []
    for finger in FINGERS:
        pose, direction, initial = find_object_placement(scene, side, finger)
        # A compliant six-axis test fixture holds a free object against one pad.
        # This probes contact response, not grasp retention under an endless force.
        fixture_target = pose[:3] - .0005*direction
        obj_id = scene.model.body("object").id
        translation_kp = 20.0
        translation_kd = 2*np.sqrt(translation_kp*scene.model.body_mass[obj_id])
        rotation_kp = .001
        rotation_kd = 2*np.sqrt(rotation_kp*np.max(scene.model.body_inertia[obj_id]))
        peak_depth = peak_force = peak_error = peak_velocity = 0.0
        contact_steps = tail_contact_steps = 0
        first = worst = last = []
        peak_fixture_force = 0.0
        for step in range(500):
            rotation_error = np.zeros(3)
            mujoco.mju_subQuat(rotation_error, pose[3:], scene.data.qpos[-4:])
            force = np.r_[translation_kp*(fixture_target-scene.data.qpos[-7:-4]) -
                          translation_kd*scene.data.qvel[-6:-3],
                          rotation_kp*rotation_error-rotation_kd*scene.data.qvel[-3:]]
            peak_fixture_force = max(peak_fixture_force, float(np.linalg.norm(force[:3])))
            scene.set_ext_force_on_obj(force)
            scene.control_hand_step(1)
            hits = [c for c in contacts(scene) if "object" in (c["body1"], c["body2"])]
            if hits:
                contact_steps += 1
                tail_contact_steps += step >= 400
                if not first:
                    first = hits
                depth = max(0.0, -min(c["distance_m"] for c in hits))
                if depth > peak_depth:
                    peak_depth, worst = depth, hits
                peak_force = max(peak_force, max(c["normal_force_n"] for c in hits))
            peak_error = max(peak_error, float(np.max(np.abs(scene.data.qpos[7:29]))))
            peak_velocity = max(peak_velocity, float(np.linalg.norm(scene.data.qvel[-6:-3])))
            if not np.isfinite(scene.data.qpos).all() or not np.isfinite(scene.data.qvel).all():
                raise AssertionError("Non-finite state in object contact")
            last = hits
        final_speed = float(np.linalg.norm(scene.data.qvel[-6:-3]))
        final_angular_speed = float(np.linalg.norm(scene.data.qvel[-3:]))
        warnings = scene.data.warning.number.tolist()
        scene.set_ext_force_on_obj(np.zeros(6))
        scene.control_hand_step(125)
        results.append(dict(
            shape=shape, finger=finger, object_pose=pose.tolist(),
            fixture=dict(type="compliant_pose_fixture", target_position_m=fixture_target.tolist(),
                         translation_kp_n_m=translation_kp, translation_kd_ns_m=float(translation_kd),
                         rotation_kp_nm_rad=rotation_kp, rotation_kd_nms_rad=float(rotation_kd),
                         peak_force_n=peak_fixture_force),
            duration_s=2.0, contact_steps=contact_steps, tail_contact_fraction=tail_contact_steps/100,
            initial_contacts=initial, first_contacts=first, deepest_contacts=worst, final_contacts=last,
            peak_penetration_m=peak_depth, peak_normal_force_n=peak_force,
            peak_joint_error_rad=peak_error, peak_object_speed_m_s=peak_velocity,
            final_object_speed_m_s=final_speed, final_object_angular_speed_rad_s=final_angular_speed,
            release_speed_m_s=float(np.linalg.norm(scene.data.qvel[-6:-3])),
            warnings=warnings, release_warnings=scene.data.warning.number.tolist(),
        ))
    return results
