# Recording Pipeline: CheatCode → Training Dataset

This document describes how to record 100+ demonstration episodes using CheatCode as the autonomous demonstrator, producing a lerobot-compatible dataset for ACT training.

## Overview

```
CheatCodeRecorder policy (in eval container)
    ↓ per timestep: observation (26D state + 3 cameras) + action (tcp_velocity 7D)
    ↓
/tmp/aic_dataset/episode_NNNNNN/
    frames.npz   left/  center/  right/  meta.json
    ↓
convert_to_lerobot.py
    ↓
HuggingFace LeRobot dataset → lerobot-train → ACT model → RunACT
```

## Data Format

### Observation (26D state)
| Index | Field |
|-------|-------|
| 0–2   | TCP position (x, y, z) |
| 3–6   | TCP orientation (qx, qy, qz, qw) |
| 7–9   | TCP linear velocity (x, y, z) |
| 10–12 | TCP angular velocity (x, y, z) |
| 13–18 | TCP error (6D) |
| 19–25 | Joint positions (7 joints) |

### Action (7D, matches RunACT output)
`[vx, vy, vz, wx, wy, wz, 0.0]` — cartesian twist in `base_link` frame.  
Sourced from `controller_state.tcp_velocity` at each timestep.

### Images
Three cameras at 25% scale (288×256 px):
- `observation.images.left_camera`
- `observation.images.center_camera`
- `observation.images.right_camera`

### Control Rate
20 Hz (0.05s per step). Each episode ≈ 530 frames (5s approach + ~21s insertion).

## Trial Types (3 per engine run, cycling)

| Type | Cable | Target | Initial XY error |
|------|-------|--------|-----------------|
| A | SFP → sfp_tip | nic_card_mount_0 / sfp_port_0 | ~−12mm, −2mm |
| B | SFP → sfp_tip | nic_card_mount_1 / sfp_port_0 | ~−12mm, +38mm |
| C | SC  → sc_tip  | sc_port_1 / sc_port_base | ~−104mm, +28mm |

## Files

| File | Purpose |
|------|---------|
| `aic_example_policies/aic_example_policies/ros/CheatCodeRecorder.py` | Recording policy |
| `scripts/gen_recording_config.py` | Generate N-trial engine config |
| `scripts/run_batch_record.sh` | Orchestrate full recording session |
| `scripts/convert_to_lerobot.py` | Convert raw data → LeRobot dataset |
| `docker/docker-compose.record.yaml` | Docker Compose override for recording |

## Quick Start

### 1. Single Episode Test

```bash
cd ~/ws_aic/src/aic

# Build image with CheatCodeRecorder
sudo docker build -f docker/aic_model/Dockerfile -t my-solution:v1 .

# Generate a 1-repeat (3-trial) config for testing
python3 scripts/gen_recording_config.py --n_repeats 1 --out recording_config.yaml

# Start eval container with recording config
sudo docker compose \
    -f docker/docker-compose.yaml \
    -f docker/docker-compose.record.yaml \
    up -d eval

# Wait for Gazebo to init (~60s), then run recorder
sleep 60
sudo docker exec -it aic-aic-eval-1 bash -c "
    source /ws_aic/install/setup.bash
    RECORD_OUTPUT_DIR=/tmp/aic_dataset \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ros2 run aic_model aic_model \
        --ros-args -p use_sim_time:=true \
        -p policy:=aic_example_policies.ros.CheatCodeRecorder
"

# Inspect output
sudo docker exec aic-eval-1 ls -la /tmp/aic_dataset/
```

### 2. Full Batch (111 episodes)

```bash
bash scripts/run_batch_record.sh 37 /tmp/aic_dataset
```

### 3. Copy & Convert

```bash
sudo docker cp aic-eval-1:/tmp/aic_dataset ./aic_dataset

pixi run python scripts/convert_to_lerobot.py \
    --src ./aic_dataset \
    --repo_id <your-hf-username>/aic_cheatcode_demos \
    --push_to_hub
```

## Engine Reset Behavior

- After each trial, engine automatically calls `reset_after_trial()`:
  - Homes robot to `home_joint_positions`
  - Deletes spawned task board + cables
  - Deactivates/reactivates model node
- No manual intervention needed between trials
- Container stays alive during all trials (`shutdown_on_aic_engine_exit:=false` in override)

## Notes

- `ground_truth:=true` is required for CheatCode/CheatCodeRecorder (needs TF ground truth)
- Dataset lives at `RECORD_OUTPUT_DIR` inside container; default `/tmp/aic_dataset`
- Each episode directory is auto-numbered: `episode_000000`, `episode_000001`, …
- Failed episodes (policy returns `False`) still save with `success: false` in `meta.json`
