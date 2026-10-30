# Single-hand Learning formatter validation — 2026-10-29

`right_full` metadata now loads without nonexistent left-hand or dummy-arm joints.
The source remains DexLearn's two-slot representation: for Shadow, each slot
contains `[xyz, quaternion_wxyz, 22 finger joints]`, making 58 values per stage.
All three converted stages contain exactly the original first 29 values.
The unused slot is not assigned invented joint names or used as left-hand kinematics.

## Changes

- `src/task/convert_format.py`: accept metadata containing either hand; validate
  the two-slot dimensions using the available hand width; return `None` for an
  absent side; reject nonfinite qpos; require right-hand metadata for single-hand
  conversion; retain both hands and both arms as requirements for dual-hand IK.
- `test/test_learning_single_hand.py`: right-only conversion of all stages,
  grasp-type filtering, unchanged object pose/scale, bimanual splitting and
  joint assembly in metadata order, left-only parsing without accidental right
  conversion, invalid dimensions/nonfinite data, and missing dual metadata.
- No runner defaults, source samples, metadata, checkpoints, or DexLearn files
  were changed for this task. Existing failed-run logs were retained.

## Tests

```bash
.venv/bin/python -m unittest test.test_learning_single_hand \
  test.test_bimanbodex_variable_candidates test.test_public_contract
```

13 tests passed. Bimanual joint assembly is regression-tested with mocked IK;
no new bimanual MuJoCo dataset evaluation was run.

## Actual MuJoCo results

| Run | Formatted | Evaluated | Success | Initial penetration failures | Physical rollouts |
| --- | ---: | ---: | ---: | ---: | ---: |
| `learned_right_full_smoke_v1_right_full_shadow` | 20 | 20 | 3 (15%) | 9 | 11 |
| `learned_right_full_100k_v1_right_full_shadow` | 6,720 | 6,720 | 331 (4.9256%) | 3,230 | 3,490 |

The full run used 10 objects and 672 scenes. No samples were filtered during
formatting, skipped as already evaluated, or missing evaluation results.
There were no evaluation exceptions, nonfinite failures, or tiny convex mesh
skips. Initial penetration failures are included in the denominator, not skipped.
Rollout counts distinguish those failures using the evaluator's `(100, 100)`
delta sentinel. Remaining unsuccessful physical rollouts: 3,159.

All 6,720 converted files were checked against their source: 22 metadata joint
names, 29 finite values in each stage, exact equality to the source right-hand
slot, and unchanged scene object pose and scale. SHA-256 verification confirmed
all 6,720 source samples, the checkpoint, and source metadata remained unchanged
through the full conversion/evaluation run (6,722 protected files).

## Reproduction

Run from `HUGS-DexGraspBench` with `HUGS_DATASET_ROOT=$HOME/hugs-dataset`:

```bash
.venv/bin/python script/process_learning_grasp_types.py \
  --hand shadow --run-name learned_right_full_100k_v1 \
  --learning-path ../HUGS-DexLearn/output/shadowMulti_robotMultiHierar_right_full_100k_v1/tests/step_050000/shadowMulti \
  --grasp-type right_full --n-worker 8 \
  --format-arg n_worker=4 \
  --format-arg +task.learning_metadata_group=right_full
```

For the smoke run, use `--run-name learned_right_full_smoke_v1 --max-num 20
--n-worker 4 --format-arg n_worker=2`, first with `--stage format`, then with
`--stage eval`. Existing outputs require care: the runner offers deletion;
this run retained existing full-run logs by answering `n`.

Actual runner pose adjustments were pregrasp `2.0`, squeeze `1.0` (the task
brief's historical squeeze `0.4` does not match current source). The current
simulation settings include initial penetration threshold 0.01 m, translation
threshold 0.05 m, angle threshold 15 degrees, and approach phase disabled.
The 4.9256% result is **not directly comparable** to the earlier BODex 50.71%:
object/scene selection, pose adjustments, and the complete simulation protocol
must be matched before making that comparison.

## Retained artifacts

Full results: `output/learned_right_full_100k_v1_right_full_shadow/`.
Its `validation/` directory contains `results.json` (including per-object
counts), source SHA-256 manifest, unit-test log, smoke format/eval logs,
full-run console log, and audit script. Hydra configurations and stage logs
are also retained under each run's `log/` directory.
