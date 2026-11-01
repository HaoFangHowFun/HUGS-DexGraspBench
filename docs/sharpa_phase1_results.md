# Sharpa Phase 1 measured results

Generated from `docs/sharpa_validation.json` after the final 64-part palm model and
Sharpa joint-limit/armature changes. MuJoCo version: 3.6.0.
Source ROS commit: `85d02e0275e8bfb55a9402f21dba08cec7b3ab97`.

**Scripted acceptance passed.** This is readiness for controlled single-hand trials,
not certification of unrestricted full grasp evaluation. No self-collision exclusions
were added, no canonical joint-zero/ordering/kinematics changed, and no Shadow pipeline
files changed.

| Hand | Representative motions | Peak tracking error | Worst settled error | 44 boundary commands | Maximum limit violation | Object tests | Maximum object penetration |
| --- | --- | --- | --- | --- | --- | --- | --- |
| right | 10/10 | 1.228° | 2.16e-08 rad | 34 tracked / 10 contact-limited | 0.0193° | 15/15 | 0.281 mm |
| left | 10/10 | 1.228° | 2.94e-08 rad | 34 tracked / 10 contact-limited | 0.0207° | 15/15 | 0.281 mm |

Both final canonical-zero holds have no contacts or drift. The original single-hull
Bench attachment had palm/thumb contact at about 2.17 mm penetration. The temporary
-10° pose is retained only as a separate historical hold regression.

All representative motions, boundary commands, deep-closing probes, and object
contact/release tests completed without numerical warnings. The 10 contact-limited
boundary commands per hand are not presented as successful free motions. Deep
closing still produces real finger-finger contact, and the controller is correctly
prevented from exactly reaching that colliding target.

## Geometry fidelity and residual contacts

| Hand | Palm parts | Source volume missed by union | Extra union volume | Static configurations | Distinct worst-case contact pairs |
| --- | --- | --- | --- | --- | --- |
| right | 64 | 0.382% | 0.867% | 198 | 51 |
| left | 64 | 0.510% | 0.625% | 198 | 51 |

All sampled final palm contacts were classified as source-geometry overlaps.
For the other meshes, each hand has two residual convex-hull artifact pairs at
large MCP lateral angles (the affected poses also have other genuine inter-finger
collisions). These were retained and documented rather than broadly disabled:

- `child-right_middle_DP/child-right_ring_MP` at `right_middle_MCP_AA=0.3491 rad`: 0.373 mm proxy penetration; source intersection 0 mm³.
- `child-right_ring_DP/child-right_pinky_MP` at `right_ring_MCP_AA=0.3491 rad`: 0.529 mm proxy penetration; source intersection 0 mm³.
- `child-left_middle_DP/child-left_ring_MP` at `left_middle_MCP_AA=0.3491 rad`: 0.374 mm proxy penetration; source intersection 0 mm³.
- `child-left_ring_DP/child-left_pinky_MP` at `left_ring_MCP_AA=0.3491 rad`: 0.529 mm proxy penetration; source intersection 0 mm³.

The scans classify sampled occurrences, not all combinations of 22 joints.
Diagnostic STL seam/hole closures and their volume changes are recorded in JSON.

## Contact stability and scope

All 30 pad/object fixture tests retained contact throughout the final hold window.
The worst held-object speed was 0.371 mm/s;
the highest post-release speed was 0.861 cm/s.
Maximum contact-loaded joint error was 0.0250°.

These tests use free objects supported by a compliant pose fixture, followed by
unforced release. They do not demonstrate grasp retention, lifting, gravity/load
resistance, floor contact, calibrated materials, learned-result conversion or
dual-hand evaluation. The source meshes and other untested poses still require
normal collision checks; the JSON intentionally reports
`full_grasp_evaluation_ready=false`.

See `sharpa_collision_validation.md` for exact reproduction commands, settings,
thresholds and the geometric rationale. The existing single-hand formatter
regression tests also passed (4 tests).
