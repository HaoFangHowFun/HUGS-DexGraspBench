# BODex vs DexLearn: matched test-scene benchmark

Date: 2026-10-29

BODex achieved **2759/6720 (41.06%)**; DexLearn achieved **331/6720 (4.93%)** on the same 10 held-out objects and 672 scene configurations. This is a scene/evaluation-budget/protocol matched comparison, not an equal-input-information or equal-compute experiment. The previous BODex 50.71% is not used as the matched baseline.

## Results

| Metric | DexLearn | BODex matched |
|---|---:|---:|
| Held-out objects / scenes | 10 / 672 | 10 / 672 |
| Saved candidates per scene | 10 | 10 |
| Evaluated candidates | 6720 | 6720 |
| Success | 331 | 2759 |
| Overall success | 4.93% | 41.06% |
| Initial penetration failures | 3230 (48.07%) | 16 (0.24%) |
| Physical rollouts | 3490 | 6704 |
| Conditional rollout success | 9.48% | 41.15% |
| Scenes with >=1 success / 672 | 211 (31.40%) | 538 (80.06%) |
| Missing evaluations | 0 | 0 |

All 6,720 BODex candidates were retained through formatting and accounted for in evaluation. No task=collect success-only filtering was used. No evaluation exceptions or tiny convex mesh skips were reported. Initial penetration failures remain in the denominator. Counts of physical rollouts exclude the evaluator's explicit `(delta_pos, delta_angle) = (100, 100)` initial-penetration sentinel.

## Per-object results

| Object ID | Candidates each | DexLearn success | BODex success | DexLearn initial penetration | BODex initial penetration |
|---|---:|---:|---:|---:|---:|
| `core_camera_98fc1afc8dec9773b10c2418bc64b141` | 960 | 55 (5.73%) | 324 (33.75%) | 508 | 2 |
| `core_cellphone_d7c350e5d29074c9b4c216fa1de7d5a1` | 600 | 13 (2.17%) | 224 (37.33%) | 254 | 0 |
| `core_jar_e1835d106d89c69ef1f3019850abf317` | 480 | 15 (3.12%) | 202 (42.08%) | 240 | 0 |
| `ddg_bigbird_chewy_dipps_chocolate_chip` | 360 | 13 (3.61%) | 113 (31.39%) | 164 | 0 |
| `ddg_gd_mushroom_poisson_004` | 960 | 38 (3.96%) | 466 (48.54%) | 412 | 4 |
| `ddg_gd_tetra_pak_poisson_004` | 720 | 45 (6.25%) | 248 (34.44%) | 360 | 0 |
| `ddg_ycb_072_a_toy_airplane` | 840 | 64 (7.62%) | 489 (58.21%) | 387 | 3 |
| `mujoco_Canon_Ink_Cartridge_Green_6` | 600 | 28 (4.67%) | 231 (38.50%) | 271 | 5 |
| `sem_SoapBar_b3718bdc7025004813f5ec8466b8a488` | 480 | 13 (2.71%) | 186 (38.75%) | 245 | 0 |
| `sem_ToiletPaper_dbe344f40aae59e6a9944a6bf3d6e5a2` | 720 | 47 (6.53%) | 276 (38.33%) | 389 | 2 |

## Uncertainty

Paired, object-stratified scene bootstrap: 5,000 replicates, seed 123. Resample scenes within each object and retain all 10 candidates of a sampled scene together. Conditional 95% intervals: DexLearn 4.32%–5.58%; BODex 38.72%–43.39%. BODex minus DexLearn: 36.13 percentage points, interval [33.97, 38.27] points.

These intervals condition on these 10 objects and saved candidate batches. They do not treat 6,720 grasps as independent trials, estimate unseen-object generalization, or include synthesis rerun variability. Poses of an object can remain correlated across scenes.

## Scene and conversion audit

- Manifest derived from every saved DexLearn `scene_path`, canonicalized, deduplicated and sorted. All paths exist; each object is in `valid_split/test.json` and absent from train. Every scene has exactly 10 DexLearn samples.
- Smoke scenes: deterministic seed 123, two scenes per object, 20 total. Smoke generated/formatted/evaluated 200 candidates, 68 successful (34%), zero initial-penetration failures, 16 successful scenes.
- Full synthesis: 672 raw files, each `(1, 10, 3, 29)`; exact scene-set equality with manifest; no duplicates, missing or extra scenes, or nonfinite stage values. No height/scale filtering or global slicing.
- Raw BODex mesh paths, scale and pose were compared against the original scenes. Every converted stage equals its original candidate stage exactly; joint names and scene object pose/scale are preserved. Each scene has exactly candidate indices 0–9, with a corresponding evaluation file for every formatted record.
- SHA-256 audit: 21184 protected paths unchanged, covering source DexLearn samples/checkpoint/metadata, original 672 scene configs, and the pre-existing learned evaluation directory.

Manifest SHA-256:

- `scenes_672.txt`: `222739efbd83b1f65115dab2237b5beaf5479306bd15345dfe43888a9c090bd8`
- `scenes_smoke_20.txt`: `8df340821022b23562f5a3c9d59bd45d9aa63a609a469ffe50d2af1a0acd2d20`

## Simulation parity and stage semantics

Saved Hydra configs were compared field by field: the complete `hand` config, `setting`, and all `task` settings match (only experiment-dependent debug output paths excluded). Runtime output paths and `skip` differ; the new BODex runs skipped zero already-evaluated candidates. Both use `tabletopMocapEval` and `assets/hand/shadow/right_hand_v2.xml` with the same joint remapping and control implementation.

- Pregrasp extrapolation 2.0; squeeze extrapolation 1.0; global z offset 0.
- Initial penetration threshold 0.01 m; translation success threshold 0.05 m; rotation threshold 15 degrees; approach phase disabled.
- Same density policy `scale_power` (recorded mass and density also matched exactly for every paired scene): reference 700 kg/m³ at scale 0.06, exponent -2, mass cap 0.5 kg; simulation friction `[0.8, 0.02]`.
- Same 0.004 s timestep; pregrasp→grasp, grasp→squeeze and squeeze→lift each run 25 outer × 10 inner steps (1 s), 0.1 m lift, downward external force `10 * object_mass`. Evaluation entry-point NumPy/Python seed is 12.

The evaluator extrapolates mocap translations/finger positions about grasp while preserving each stage's wrist quaternion. This is defined for both inputs. BODex optimizes pregrasp and grasp, then constructs squeeze with grasp wrist pose and fingers `2*grasp - pregrasp`; squeeze ratio 1.0 preserves that construction. DexLearn predicts all three stages. These method-specific trajectory semantics remain different despite matched evaluator settings; this run does not isolate pose adjustment or stage construction as a causal explanation.

BODex reads complete mesh geometry and scene transforms, using surface initialization; DexLearn here consumes complete point clouds. Matching 10 evaluated candidates per scene does not equalize information, optimizer effort, model training cost or proposal selection. The task brief reports DexLearn `test_grasp_num=20`, `test_topk=10`, but the experiment's current `.hydra` files were overwritten by a later visualization launch (whose config says 100 proposals). The preserved sampling console verifies 10 objects/672 scenes but not the proposal override. Therefore only the saved/evaluated count of 10 is independently verified; do not treat the proposal budget as audited.

## Implementation and generation settings

BODex changes are limited to `cfg/task/synthesis.yaml`, `src/task/synthesis.py` and focused tests in `tests/test_public_contracts.py`. New `task.scene_source.manifest_path` reads exact .npy paths (relative entries resolve against the manifest), validates existence, rejects duplicate canonical paths and empty manifests, and forbids combining manifests with scene slicing or object-scale filtering. Existing glob selection remains the fallback.

Experiment-specific `manifest/baseline.yml` overrides only `sim_shadow/tabletop_full.yml: seed_num=10`. The historical `heur_multi_shadow.yml` was not applied because its scale filters conflict with exact scene coverage; its right-full method parameters are otherwise the same tabletop_full configuration. Suite `sim_shadow.yml`, grasp type `3_right_full`, surface initialization, grouping `object_scale`, solver reuse `batch`, parallel worlds 20, one GPU, fixed synthesis module seed 123. Full plan: 34 batches, last batch 12 scenes; measured synthesis execution 181.561 s, excluding launch/planning. MuJoCo evaluation took approximately 84 s. All returned 10 seeds are saved, without success-only selection.

Focused tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ../HUGS-DexLearn/.venv/bin/python -m pytest -q tests/test_public_contracts.py -k "scene_manifest or manifest_selection"` from BODex: **2 passed**. The BODex venv has no pytest; the existing DexLearn pytest environment ran these CPU/AST tests. Dry plans confirmed smoke 20/200 and full 672/6,720 with zero scene filtering.

## Reproduction and artifacts

Persistent experiment root: `HUGS-DexGraspBench/output/bodex_matched_test_v1/`.

From `HUGS-DexGraspBench`, run stages in this order (each `run_stage.py` command records its exact argv, environment, cwd and logfile in `commands.jsonl`):

```bash
.venv/bin/python output/bodex_matched_test_v1/scripts/run_stage.py smoke dry
.venv/bin/python output/bodex_matched_test_v1/scripts/run_stage.py smoke synthesis
.venv/bin/python output/bodex_matched_test_v1/scripts/run_stage.py smoke format
.venv/bin/python output/bodex_matched_test_v1/scripts/run_stage.py smoke eval
.venv/bin/python output/bodex_matched_test_v1/scripts/audit.py smoke
# Full run only after smoke audit passes:
.venv/bin/python output/bodex_matched_test_v1/scripts/run_stage.py full dry
.venv/bin/python output/bodex_matched_test_v1/scripts/run_stage.py full synthesis
.venv/bin/python output/bodex_matched_test_v1/scripts/run_stage.py full format
.venv/bin/python output/bodex_matched_test_v1/scripts/run_stage.py full eval
.venv/bin/python output/bodex_matched_test_v1/scripts/audit.py full
.venv/bin/python output/bodex_matched_test_v1/scripts/compare.py
```

The completed artifacts already exist. For independent reruns use new experiment/run IDs; synthesis/evaluation support skip-based resume, while reformatting would overwrite converted files in the selected new run. No existing baseline or learned run was deleted or overwritten in this experiment.

- `manifest/`: deterministic scene/object lists, verification JSON, baseline override, protected SHA-256 manifest.
- `synthesis_smoke/`, `synthesis_full/`: generated candidates, plan, Hydra launch configs.
- `logs/`, `commands.jsonl`: retained dry-run, test, synthesis, format and evaluation logs/commands.
- `resolved_configs/`: resolved BODex suite/manip dictionaries, learned and BODex eval configs, evaluator/hand asset checksums and repository revisions. Copied DexLearn experiment Hydra files are explicitly from visualization, not a recovered sampling config.
- `audit_smoke.json`, `audit_full.json`, `comparison.json`: scene/candidate audits, per-object metrics, paired bootstrap intervals.
- Benchmark outputs: sibling directories `bodex_matched_test_v1_smoke_right_full_shadow/` and `bodex_matched_test_v1_full_right_full_shadow/`, each with `graspdata/`, `evaluation/`, `succgrasp/`, and saved Hydra logs.

## Next steps

Inspect paired scenes with different outcomes, separating initial-penetration failures from failed completed rollouts. For diagnosis, evaluate both methods under additional matched pregrasp settings in new runs; do not replace these results. Archive future DexLearn sampling overrides separately from visualization configs, and record proposal counts and generation wall time before claiming equal-compute performance.
