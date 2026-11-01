# Sharpa MuJoCo model validation

## Current Phase 1 implementation

See [Collision/contact validation](sharpa_collision_validation.md) for the current
simulation profile, collision rationale, commands and readiness limits, and
[Measured results](sharpa_phase1_results.md) for the latest test summary.

The Bench palm collider now uses 64 convex parts. No self-collision exclusions were
added. Canonical zero is tested directly; the temporary -10-degree pose is isolated
to the Phase 0 regression. Run `scripts/validate_sharpa_phase1.py` for the complete
suite and `docs/sharpa_validation.json` output. The smaller
`scripts/validate_sharpa_models.py` writes `docs/sharpa_model_validation.json`.

## Historical Phase 0 notes

The following records the original single-hull model and its findings. The current
Phase 1 changes and measurements supersede its collision/readiness conclusions.

Place `HUGS-Sharpa-Assets` beside this repository, then run:

```bash
.venv/bin/python scripts/build_sharpa_models.py
.venv/bin/python scripts/validate_sharpa_models.py
```

The builder derives `assets/hand/sharpa_wave/{right,left}.xml` from the source
URDFs. All meshes remain in the independent asset directory. Hand configs are
`config/hand/sharpa_{right,left}.yaml`; simulation-only gains are in
`config/sharpa_validation.yaml`. These values are provisional, not hardware gains.
Both hands use the flange root and the source 22-joint canonical order, with
independent position actuators bounded by the URDF position and effort limits.
URDF velocity limits are not enforced by MuJoCo position actuators.

The validator checks geometry, source FK, actuator mapping/limits, named body
groups, RobotKinematics, and the actual MjHO free-root attachment. It tests a
nonuniform command to detect index permutations, then records a 250-step zero-pose
baseline and tests the configured initial pose for 1,250 steps (5 seconds), with a
synthetic box far from the hand. No dataset is loaded.
Results are in `docs/sharpa_validation.json`.

## Initial pose

`config/sharpa_validation.yaml` sets `right_thumb_CMC_AA` and
`left_thumb_CMC_AA` to **-10 degrees** (`-0.17453292519943295` radians), with all
other joints at zero. The validator sets both the initial joint state and the
position-controller targets to this pose. This is an explicit simulation start
pose, not a change to the URDF zero, joint limits, or incoming grasp poses.

Both attached hands hold this pose for 5 seconds with zero contacts, zero observed
joint tracking error, and no numerical warnings. Gravity remains disabled. The
report retains the zero-pose baseline separately from `initial_pose_hold`, so
the intentional -10-degree angle is not counted as drift.

**Not yet calibrated for grasp evaluation:** the original attached zero pose has
a palm/thumb-MC contact (about 2.17 mm reported penetration), and its zero-command
baseline drifts to roughly 0.136 rad. The standalone fixed-root model also reports
other adjacent-link contacts. The adjusted start pose avoids the attached contact;
it does not establish collision-free motion over the full joint range. Review
collision filtering, convex mesh contacts, gains and the provisional flange +Z
approach axis before reporting grasp quality metrics. Source meshes and geometry
were preserved; no exclusions were added to suppress the finding.

The existing Learning formatter still assumes Shadow/LEAP-style metadata names
and hand-family rules. These configs establish model validation, not Sharpa learned
result conversion or dual-hand evaluation. See the sibling asset directory's
`SHADOW_AUDIT.md` for the complete audit and remaining integration gates.
