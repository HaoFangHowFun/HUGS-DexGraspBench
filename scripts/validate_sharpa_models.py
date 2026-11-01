"""Exercise Sharpa FK, controller limits and the actual bench MjHO attachment."""
import json
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))
from util.hand_util import MjHO, RobotKinematics


def check_hold(scene, hand_q, object_q, steps):
    """Reset both state and controller target, then measure error throughout a hold."""
    scene.reset_pose_qpos(hand_q, object_q)
    np.testing.assert_allclose(scene.data.ctrl, hand_q[7:], rtol=0, atol=1e-12)
    contacts = [{
        "body1": scene.model.body(scene.model.geom_bodyid[c.geom1]).name,
        "body2": scene.model.body(scene.model.geom_bodyid[c.geom2]).name,
        "distance_m": float(c.dist),
    } for c in scene.data.contact]
    peak_contacts = scene.data.ncon
    peak_error = 0.0
    for _ in range(steps):
        mujoco.mj_step(scene.model, scene.data)
        if not np.isfinite(scene.data.qpos).all() or not np.isfinite(scene.data.qvel).all():
            raise AssertionError("Non-finite simulation state")
        if np.any(scene.data.warning.number):
            raise AssertionError("MuJoCo emitted simulation warnings")
        peak_contacts = max(peak_contacts, scene.data.ncon)
        peak_error = max(peak_error, float(np.max(np.abs(scene.data.qpos[7:29] - hand_q[7:]))))
    return dict(
        steps=steps, duration_s=steps * scene.model.opt.timestep,
        initial_joint_qpos=hand_q[7:].tolist(), target_joint_qpos=hand_q[7:].tolist(),
        initial_contacts=contacts, peak_contact_count=peak_contacts,
        peak_joint_tracking_error_rad=peak_error,
        final_joint_tracking_error_rad=float(np.max(np.abs(scene.data.qpos[7:29] - hand_q[7:]))),
    )


def main():
    config = yaml.safe_load((REPO / "config/sharpa_validation.yaml").read_text())
    assets = (REPO / config["asset_root"]).resolve()
    sys.path.insert(0, str(assets / "scripts"))
    from validate_models import validate_side, require
    reports = []
    with tempfile.TemporaryDirectory(prefix="sharpa-object-") as directory:
        mesh_dir = Path(directory) / "urdf/meshes"
        mesh_dir.mkdir(parents=True)
        trimesh.creation.box(extents=[0.03]*3).export(mesh_dir / "convex_piece_0.obj")
        for side in ("right", "left"):
            hand = yaml.safe_load((REPO / f"config/hand/sharpa_{side}.yaml").read_text())
            xml = REPO / hand["xml_path"]
            # Shared assets retain exact source geometry. Bench replaces only the
            # concave palm collision geom with documented convex parts.
            result = validate_side(side)
            kin = RobotKinematics(str(xml))
            model = kin.mj_model
            reference = mujoco.MjModel.from_xml_path(str(assets / "urdf" / f"{side}_sharpa_wave_with_flange.urdf"))
            require([model.body(i).name for i in range(model.nbody)] ==
                    [reference.body(i).name for i in range(reference.nbody)], "Body hierarchy changed")
            for attribute in ("body_parentid", "body_pos", "body_quat", "jnt_pos", "jnt_axis", "jnt_range"):
                np.testing.assert_allclose(getattr(model, attribute), getattr(reference, attribute), rtol=0, atol=1e-12)
            np.testing.assert_allclose(model.body_mass, reference.body_mass, rtol=1e-5, atol=1e-12)
            np.testing.assert_allclose(model.body_inertia, reference.body_inertia, rtol=1e-5, atol=1e-12)
            # Visual geoms and every non-palm collision geom must remain intact.
            palm_id = model.body(f"{side}_hand_C_MC").id
            keep = lambda m: np.flatnonzero((m.geom_bodyid != palm_id) | ((m.geom_contype | m.geom_conaffinity) == 0))
            for attribute in ("geom_type", "geom_bodyid", "geom_contype", "geom_conaffinity", "geom_pos", "geom_quat"):
                np.testing.assert_allclose(getattr(model, attribute)[keep(model)],
                                           getattr(reference, attribute)[keep(reference)], rtol=0, atol=2e-6)
            names = result["joint_names"]
            require(model.nu == 22 and model.ntendon == 0, "Expected independent position controls")
            np.testing.assert_allclose(model.actuator_ctrlrange, model.jnt_range, atol=1e-8)
            np.testing.assert_array_equal(model.actuator_trnid[:, 0], np.arange(22))
            source = ET.parse(assets / "urdf" / f"{side}_sharpa_wave_with_flange.urdf")
            effort = [float(j.find("limit").get("effort")) for j in source.findall("joint")
                      if j.get("type") != "fixed"]
            np.testing.assert_allclose(model.actuator_forcerange, np.array([-np.array(effort), effort]).T)
            require(np.all(model.actuator_forcelimited) and np.all(model.actuator_ctrllimited),
                    "Actuator force or control limits disabled")
            for key in ("valid_body_name", "tip_body_name", "wrist_body_names"):
                for name in hand[key]:
                    require(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0,
                            f"Unknown configured body: {name}")
            scene = MjHO(directory, 1.0, 500.0, None, str(xml), hand["mocap"],
                         hand["exclude_table_contact"], [0.5, 0.01], False)
            mujoco.mj_forward(scene.model, scene.data)
            scene._init_after_first_fk()
            require(scene.model.nq == 36 and scene.model.nv == 34, "Unexpected attached scene layout")
            require(scene.get_joint_names() == ["hand_freejoint"] + names, "Attached joint order mismatch")
            hand_q = np.r_[0, 0, 0.5, 1, 0, 0, 0, np.zeros(22)]
            object_q = np.array([1, 1, 1, 1, 0, 0, 0])
            np.testing.assert_allclose(scene._qpos2ctrl(hand_q), hand_q[7:], atol=1e-12)
            # Test a nonuniform command to expose accidental index permutations.
            probe = hand_q.copy()
            probe[7:] = np.linspace(0.01, 0.1, 22)
            np.testing.assert_allclose(scene._qpos2ctrl(probe), probe[7:], atol=1e-12)
            baseline = check_hold(scene, hand_q, object_q, 250)
            initial_positions = config["initial_joint_positions"][side]
            require(set(initial_positions) <= set(names), "Unknown initial-pose joint name")
            hand_q[7:] = [initial_positions.get(name, 0.0) for name in names]
            require(np.isfinite(hand_q).all(), "Non-finite initial pose")
            require(np.all(hand_q[7:] >= model.jnt_range[:, 0]) and
                    np.all(hand_q[7:] <= model.jnt_range[:, 1]), "Initial pose exceeds joint limits")
            steps = int(config["hold_steps"])
            require(steps > 0, "hold_steps must be positive")
            require(np.isclose(scene.model.opt.timestep, config["timestep"]), "Timestep mismatch")
            hold = check_hold(scene, hand_q, object_q, steps)
            require(hold["peak_contact_count"] == 0, "Initial-pose hold has unexpected contacts")
            require(hold["peak_joint_tracking_error_rad"] < 1e-6, "Initial-pose hold drifts")
            result.update(attached_nq=scene.model.nq, attached_nu=scene.model.nu,
                          zero_pose_baseline=baseline, initial_pose_hold=hold,
                          bench_collision_geoms=int(np.count_nonzero(model.geom_contype | model.geom_conaffinity)),
                          bench_exclusions=model.nexclude)
            reports.append(result)
    output = REPO / "docs/sharpa_model_validation.json"
    output.parent.mkdir(exist_ok=True)
    output.write_text(json.dumps({"mujoco_version": mujoco.__version__, "hands": reports}, indent=2) + "\n")
    for result in reports:
        hold = result["initial_pose_hold"]
        print(f"{result['side']}: FK, 22 controls and {hold['duration_s']:g}s initial-pose hold passed; "
              f"peak contacts={hold['peak_contact_count']}, "
              f"peak joint error={hold['peak_joint_tracking_error_rad']:.3g} rad")


if __name__ == "__main__":
    main()
