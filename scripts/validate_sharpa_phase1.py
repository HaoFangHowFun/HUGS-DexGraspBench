"""Run Sharpa Phase 1 motion, geometry, limits and contact checks; exit nonzero on failure."""
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

import validate_sharpa_models
from sharpa_geometry import GeometryAudit
from sharpa_phase1 import (REPO, contacts, limit_checks, make_scene, names_and_limits,
                          object_checks, representative_poses, reset, run_motion, static_range_scan)


def settings(scene):
    model = scene.model
    collision = np.flatnonzero(model.geom_contype | model.geom_conaffinity)
    return dict(
        timestep_s=float(model.opt.timestep), integrator=int(model.opt.integrator),
        gravity_enabled=not bool(model.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_GRAVITY),
        cone=int(model.opt.cone), impratio=float(model.opt.impratio),
        noslip_iterations=int(model.opt.noslip_iterations), iterations=int(model.opt.iterations),
        joint_damping=model.dof_damping.tolist(), armature=model.dof_armature.tolist(),
        joint_limit_solref=np.unique(model.jnt_solref, axis=0).tolist(),
        joint_limit_solimp=np.unique(model.jnt_solimp, axis=0).tolist(),
        frictionloss=model.dof_frictionloss.tolist(), actuator_gain=model.actuator_gainprm[:, 0].tolist(),
        actuator_velocity_bias=model.actuator_biasprm[:, 2].tolist(),
        contact_solref=np.unique(model.geom_solref[collision], axis=0).tolist(),
        contact_solimp=np.unique(model.geom_solimp[collision], axis=0).tolist(),
        friction=np.unique(model.geom_friction[collision], axis=0).tolist(),
        condim=np.unique(model.geom_condim[collision]).tolist(), exclusions=int(model.nexclude),
    )


def raw_attachment(directory, assets, side):
    spec = mujoco.MjSpec.from_file(str(assets / "urdf" / f"{side}_sharpa_wave_with_flange.urdf"))
    spec.compile()
    root = ET.fromstring(spec.to_xml())
    root.find("compiler").set("meshdir", str(assets / "meshes" / side))
    xml = Path(directory) / f"raw_{side}.xml"
    ET.ElementTree(root).write(xml)
    scene = make_scene(directory, xml)
    reset(scene)
    return contacts(scene)


def main():
    config = yaml.safe_load((REPO / "config/sharpa_validation.yaml").read_text())
    assets = (REPO / config["asset_root"]).resolve()
    thresholds = config["phase1_acceptance"]
    # Re-run the underlying model/FK and canonical/temporary-pose checks first.
    validate_sharpa_models.main()
    report = json.loads((REPO / "docs/sharpa_model_validation.json").read_text())
    manifest = json.loads((REPO / config["palm_decomposition"]["directory"] / "manifest.json").read_text())
    provenance = json.loads((assets / "provenance.json").read_text())
    for filename, record in provenance["copied_files"].items():
        if hashlib.sha256((assets / filename).read_bytes()).hexdigest() != record["sha256"]:
            raise ValueError(f"Canonical source copy changed: {filename}")
    failures = []
    with tempfile.TemporaryDirectory(prefix="sharpa-phase1-") as directory:
        shadow = make_scene(directory, REPO / "assets/hand/shadow/right_hand_v2.xml")
        shadow_settings = settings(shadow)
        for hand in report["hands"]:
            side = hand["side"]
            xml = REPO / f"assets/hand/sharpa_wave/{side}.xml"
            scene = make_scene(directory, xml)
            reset(scene)
            canonical_contacts = contacts(scene)
            if canonical_contacts:
                failures.append(f"{side}: canonical zero contacts")
            audit = GeometryAudit(assets, side, manifest, REPO)
            print(f"{side}: auditing source geometry and full single-joint ranges", flush=True)
            scan = static_range_scan(scene)
            for record in scan["contact_pairs"].values():
                c = record["contact"]
                record["geometry"] = audit.compare(c["body1"], c["body2"], record["qpos"])
            for record in scan["palm_contact_samples"]:
                c = record["contact"]
                record["geometry"] = audit.compare(c["body1"], c["body2"], record["qpos"])
            motions = {}
            for name, target in representative_poses(scene).items():
                motion = run_motion(scene, target)
                motion["passed"] = (
                    not any(motion["warnings"]) and not motion["self_contact_pairs"] and
                    motion["peak_tracking_error_rad"] <= thresholds["peak_tracking_error_rad"] and
                    motion["settled_tracking_error_rad"] <= thresholds["settled_tracking_error_rad"] and
                    motion["return_error_rad"] <= thresholds["settled_tracking_error_rad"] and
                    motion["peak_limit_violation_rad"] <= thresholds["joint_limit_violation_rad"])
                if not motion["passed"]:
                    failures.append(f"{side}: motion {name}")
                motions[name] = motion
            # Keep the deep closure that causes genuine inter-finger contact as a
            # positive control, rather than deleting it or disabling its collisions.
            deep_target = representative_poses(scene)["combined_closing"]/.5
            deep = run_motion(scene, deep_target)
            reset(scene, deep_target)
            deep["target_contact_geometry"] = []
            seen = set()
            for contact in contacts(scene):
                pair = (contact["body1"], contact["body2"])
                if pair in seen:
                    continue
                seen.add(pair)
                deep["target_contact_geometry"].append(dict(
                    contact=contact, geometry=audit.compare(*pair, deep_target)))
            deep["passed"] = bool(deep["self_contact_pairs"] and not any(deep["warnings"]) and
                                  any(item["geometry"]["classification"] == "source_geometry_overlap"
                                      for item in deep["target_contact_geometry"]))
            if not deep["passed"]:
                failures.append(f"{side}: deep closure must retain real self-contact")
            print(f"{side}: executing all 44 single-joint boundary commands", flush=True)
            boundary_motions = []
            names, ranges = names_and_limits(scene)
            for index, joint in enumerate(names):
                for label, angle in zip(("lower", "upper"), ranges[index]):
                    target = np.zeros(22)
                    target[index] = angle
                    item = run_motion(scene, target, return_to_zero=False)
                    item.update(joint=joint, boundary=label)
                    if item["self_contact_pairs"]:
                        reset(scene, target)
                        pairs = {(c["body1"], c["body2"]) for c in contacts(scene)}
                        item["target_contact_geometry"] = [dict(pair=pair, geometry=audit.compare(*pair, target))
                                                           for pair in sorted(pairs)]
                        item["status"] = "contact_limited"
                    else:
                        item["status"] = ("tracked" if item["settled_tracking_error_rad"] <=
                                          thresholds["settled_tracking_error_rad"] else "tracking_failure")
                    if any(item["warnings"]) or item["peak_limit_violation_rad"] > thresholds["joint_limit_violation_rad"]:
                        failures.append(f"{side}: boundary instability {joint}/{label}")
                    if item["status"] == "tracking_failure":
                        failures.append(f"{side}: unexplained boundary tracking error {joint}/{label}")
                    boundary_motions.append(item)
            limits = limit_checks(scene)
            for item in limits:
                item["passed"] = bool(not any(item["warnings"]) and
                                  item["clipping_limit_violation_rad"] <= thresholds["joint_limit_violation_rad"] and
                                  abs(item["final_qpos"]-scene.model.jnt_range[
                                      next(i for i in range(scene.model.njnt) if scene.model.joint(i).name.endswith("index_PIP")),
                                      0 if item["boundary"] == "lower" else 1]) <= thresholds["settled_tracking_error_rad"])
                if not item["passed"]:
                    failures.append(f"{side}: limit {item['boundary']}")
            object_results = []
            for shape in ("box", "sphere", "cylinder"):
                print(f"{side}: {shape} contact tests", flush=True)
                object_scene = make_scene(directory, xml, shape)
                for item in object_checks(object_scene, side, shape):
                    item["passed"] = (
                        not any(item["warnings"] + item["release_warnings"]) and
                        item["tail_contact_fraction"] >= thresholds["contact_tail_fraction"] and
                        item["peak_penetration_m"] <= thresholds["contact_penetration_m"] and
                        item["peak_joint_error_rad"] <= thresholds["peak_tracking_error_rad"] and
                        item["final_object_speed_m_s"] <= thresholds["contact_final_speed_m_s"] and
                        item["release_speed_m_s"] <= thresholds["release_speed_m_s"] and
                        item["peak_normal_force_n"] > 0)
                    if not item["passed"]:
                        failures.append(f"{side}: {shape}/{item['finger']} contact")
                    object_results.append(item)
            names, ranges = names_and_limits(scene)
            # Effective welded parent groups explain source fixed-root vs free-root filtering.
            hierarchy = [{"body": scene.model.body(i).name,
                          "parent": scene.model.body(scene.model.body_parentid[i]).name,
                          "weld_group": scene.model.body(scene.model.body_weldid[i]).name}
                         for i in range(scene.model.nbody)]
            hand["phase1"] = dict(
                model_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(), source_commit=provenance["source_commit"],
                canonical_zero_contacts=canonical_contacts,
                original_single_hull_attached_zero_contacts=raw_attachment(directory, assets, side),
                zero_pose_geometry=audit.zero_pairs(side), diagnostic_mesh_closures=audit.repairs,
                body_hierarchy=hierarchy, effective_settings=settings(scene),
                static_joint_range_scan=scan, motions=motions, joint_limit_tests=limits,
                single_joint_boundary_motions=boundary_motions, deep_closing_contact_probe=deep,
                object_contact_tests=object_results,
            )
            print(f"{side}: {sum(m['passed'] for m in motions.values())}/{len(motions)} motions, "
                  f"{sum(t['passed'] for t in object_results)}/{len(object_results)} object contacts passed", flush=True)
    residuals = {hand["side"]: [pair for pair, item in hand["phase1"]["static_joint_range_scan"]["contact_pairs"].items()
                                if item["geometry"]["classification"] == "convex_approximation_overlap"]
                 for hand in report["hands"]}
    report["phase1"] = dict(
        source_commit=provenance["source_commit"], thresholds=thresholds,
        decomposition=manifest, exclusions=config["self_collision_exclusions"],
        shadow_effective_settings=shadow_settings, acceptance_passed=not failures, failures=failures,
        residual_approximation_pairs=residuals,
        readiness="controlled_contact_and_motion_tests_passed" if not failures else "not_ready",
        full_grasp_evaluation_ready=False,
        scope="Representative 2-second ramps/1-second holds and compliant pad fixtures; not object retention or full grasp validation",
    )
    output = REPO / "docs/sharpa_validation.json"
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(f"Saved {output}; failures={failures}")
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
