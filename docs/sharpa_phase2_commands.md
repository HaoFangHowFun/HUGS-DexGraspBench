# Phase 2 reproduction

Run from the indicated directories. The final recorded output root is
`/home/hao-fang/hugs-workspace/benchmarks/sharpa_phase2_final`; choose a fresh
directory for a new run to preserve previous results.

```bash
export HUGS_DATASET_ROOT=/home/hao-fang/hugs-dataset
export HUGS_OBJECT_ROOT=/home/hao-fang/hugs-dataset/object
export SHARPA_DIAGNOSTIC_DIR=/home/hao-fang/hugs-workspace/benchmarks/sharpa_phase2_final
export HUGS_OUTPUT_ROOT="$SHARPA_DIAGNOSTIC_DIR/output"
export MUJOCO_GL=egl

cd /home/hao-fang/hugs-workspace/HUGS-BODex
.venv/bin/python scripts/prepare_sharpa_synthesis.py
.venv/bin/python scripts/validate_sharpa_synthesis.py

.venv/bin/python scripts/run_sharpa_smoke.py task=synthesis name=sharpa_smoke \
  manip_cfg_file=sim_sharpa/tabletop_full.yml task.suite_config=sim_sharpa.yml \
  'task.grasp_types=[3_right_full]' \
  task.scene_source.manifest_path=/home/hao-fang/hugs-workspace/benchmarks/sharpa_phase2/scenes.txt \
  task.parallel_world=1 n_worker=1

.venv/bin/python scripts/inspect_sharpa_smoke.py \
  "$HUGS_OUTPUT_ROOT/sim_sharpa/tabletop_full/sharpa_smoke/graspdata/core_bottle_1071fa4cddb2da2fc8724d5673a063a6/tabletop_ur10e/scale008_pose004_0_grasp.npy" \
  "$SHARPA_DIAGNOSTIC_DIR"
```

Inspect `candidate_grasps.png` and `candidate_validation.json` before evaluating.
The saved manifest contains exactly this one scene:

```text
/home/hao-fang/hugs-dataset/object/DGN_2k/scene_cfg/core_bottle_1071fa4cddb2da2fc8724d5673a063a6/tabletop_ur10e/scale008_pose004_0.npy
```

Its actual scale is 0.081, not a value inferred from the filename. Keep all saved
candidates, including optimizer failures and extrapolated squeeze targets, for
diagnostics. They must not be counted as successful or all-stage-valid grasps.

```bash
cd /home/hao-fang/hugs-workspace/HUGS-DexGraspBench
.venv/bin/python src/main.py task=format hand=sharpa_right \
  exp_name=sharpa_smoke_right_full save_root="$SHARPA_DIAGNOSTIC_DIR/bench" \
  task.data_name=BimanBODex \
  task.data_path="$HUGS_OUTPUT_ROOT/sim_sharpa/tabletop_full/sharpa_smoke/graspdata" n_worker=1

.venv/bin/python scripts/evaluate_sharpa_smoke.py task=eval hand=sharpa_right \
  exp_name=sharpa_smoke_right_full save_root="$SHARPA_DIAGNOSTIC_DIR/bench" \
  task.start=0 task.end=20 n_worker=1

.venv/bin/python scripts/render_sharpa_smoke.py task=eval hand=sharpa_right \
  exp_name=sharpa_smoke_right_full save_root="$SHARPA_DIAGNOSTIC_DIR/bench" n_worker=1

.venv/bin/python scripts/summarize_sharpa_smoke.py "$SHARPA_DIAGNOSTIC_DIR"
.venv/bin/python -m unittest discover -s test -p test_learning_single_hand.py
```

The diagnostic evaluator invokes the existing `tabletopMocapEval.run()` and wraps
`mujoco.mj_step` with observations after the original call. Each physics step records
contact minima and warning counters; every tenth step stores contact positions,
normals, body/geom identities and object pose. It restores the original function
after each candidate. Named fallback identities preserve unnamed Sharpa geoms.
The standard evaluator files and thresholds are unchanged.

Resolved synthesis/evaluation configs, exact synthesis config snapshots and hashes,
solver NPZ, raw/converted/evaluated NPYs, per-candidate contact JSON, and logs are
kept under the run directory. Recorded versions: MuJoCo 3.6.0, Warp 1.18.0; GPU RTX
5090. Seed 123 governs the existing synthesis runtime; the wrapper additionally
seeds trimesh surface sampling, whose default otherwise uses OS entropy. CUDA
floating-point optimization is not promised bitwise identical across machines.
