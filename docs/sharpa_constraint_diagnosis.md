# Saved-grasp constraint diagnosis

Read-only diagnosis of the 20 final Sharpa candidates and 20 Shadow control
candidates on the same bottle. No new synthesis, parameter change, contact
exclusion, or joint-limit relaxation was performed.

## Findings

| Quantity at saved grasp pose | Sharpa | Shadow |
| --- | --- | --- |
| Candidates within 0.01 rad of a hard joint limit | 0/20 | 0/20 |
| Candidates inside the optimizer's 10% joint-limit guard band | 20/20 | 9/20 |
| Median actual weighted joint-bound cost | 0.164401 | 0 |
| Maximum actual weighted joint-bound cost | 0.448078 | 0.000203 |
| Collision spheres | 998 | 50 |
| Candidates with positive world-collision cost | 15/20 | 5/20 |
| Median actual weighted world-collision cost | 0.002279 | 0 |
| Maximum actual weighted world-collision cost | 0.376447 | 0.002065 |
| Candidates with positive self-collision cost | 1/20 | 10/20 |
| Median saved grasp distance error | 41.812 mm | 0.814 mm |

The saved final-stage distance error is the largest absolute mesh-contact distance
residual among the five designated contacts for each candidate. The table takes
the median across candidates. It does not mean every finger is 41.8 mm away or
that there are no contacts anywhere on the hand. Bench reports static contacts
in 10/20 Sharpa candidates and 20/20 Shadow candidates.

The hard URDF limits are not reached. However, `gradient_grasp_fc.yml` sets bound
activation distance to 0.1, and `bound_cost.py` multiplies this by each joint's
range: the outer 10% at each end receives a quadratic penalty. Sharpa's minimum
remaining hard-limit margin is 3.814% of its joint range, compared with 9.830%
for Shadow. Sharpa abduction joints frequently enter this soft guard band (index
MCP_AA 19/20, middle 17/20, ring 12/20, pinky 20/20). This is an optimization
penalty rather than a physical hard stop. Root pose dimensions have broad bounds
of ±1000, so this finding concerns hand joint angles, not a root quaternion limit.

`PrimitiveCollisionCost.sum_distance` is true: world collision costs sum over
spheres without normalization by the sphere count. The denser Sharpa fit therefore
changes the relationship between geometric coverage and cost weighting. More
spheres does **not** imply a fixed 20× cost: only active spheres contribute, and
their distances differ. At the final Sharpa poses, the median active count is 2,
with maximum 17. This is a calibration concern, not proof of the sole cause.

The final contact strategy assigns non-contact-link spheres a 2 mm object margin
and contact-link spheres a -2 mm target margin. For Sharpa only the elastomer links
are contact links; their fixed DP finger bones remain non-contact links. Shadow's
entire distal link is a contact link. This geometric/semantic difference can make
the Sharpa finger bone oppose approach by its pad and deserves a geometric test.
It is not a reason to disable all distal collisions or self-collisions.

Most final Sharpa poses have zero self-collision cost, despite poor contact
residuals. This argues against a blanket claim that all hands are currently
penetrating themselves and stuck. It does not exclude earlier collision avoidance
steering optimization into a poor pose. Initial hand shape, selected early contact
targets, and later contact regularization remain alternative contributors.

## Interpretation and next experiment

There is evidence of active soft-limit penalties and a collision-cost/contact-link
configuration mismatch to investigate. There is not yet a controlled causal
ablation establishing that either alone causes the 41.8 mm contact residual.
Keep canonical hard limits and the Shadow evaluator fixed. Separately test a
Sharpa-local soft guard-band change, then collision fitting/cost normalization and
pad/DP contact semantics, with the same initial candidates. Report real mesh
penetration and contact distance alongside optimizer cost so that removing a
penalty cannot masquerade as a valid grasp.

The earlier sphere work verified geometry samples and pipeline execution; it did
not calibrate how a 998-sphere representation changes the optimization objective.
This is a remaining integration issue, not evidence that the physical Sharpa hand
cannot grasp the bottle.

## Reproduce

```bash
cd /home/hao-fang/hugs-workspace/HUGS-BODex
HUGS_DATASET_ROOT=/home/hao-fang/hugs-dataset \
  .venv/bin/python scripts/diagnose_smoke_constraints.py
```

This instantiates the real solver configuration, evaluates its bound, primitive
collision and self-collision costs on saved poses, and does not invoke a solve.
Detailed per-candidate and per-link costs are saved to
`benchmarks/shadow_bottle_control/constraint_diagnostics.json` in the workspace.
Cost magnitudes are weighted objective units, not millimeters or forces.
