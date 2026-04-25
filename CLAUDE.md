# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the **AI for Industry Challenge (AIC)** toolkit — a ROS 2-based framework for developing cable insertion robot policies for industrial automation. Competitors implement Python policies; the organizer-provided evaluation infrastructure runs the trials.

## Build Commands

```bash
# Build workspace (run from ~/ws_aic/, not src/aic/)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Run a policy (using Pixi environment)
pixi run ros2 run aic_model aic_model --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.WaveArm

# Launch full simulation
ros2 launch aic_bringup aic_gz_bringup.launch.py ground_truth:=false start_aic_engine:=true

# Build participant Docker image
docker build -f docker/aic_model/Dockerfile -t my-solution:v1 .

# Run full evaluation with docker-compose
docker-compose -f docker/docker-compose.yaml up
```

## Code Style

```bash
black .                          # Python formatting
clang-format -i **/*.cpp **/*.hpp  # C++ formatting (v19)
```

CI enforces these via `.github/workflows/style.yml`. Python imports use `.isort.cfg`.

## Architecture

**Two-container architecture:**
- `aic_eval` container (organizer): Gazebo simulation, engine state machine, controller, scoring
- `aic_model` container (participant): ROS 2 lifecycle node wrapper + policy implementation

**Engine state machine flow:** Model Ready → Endpoints Ready → Simulator Ready → Scoring Ready → Task Started → Task Completed

**Control flow:**
```
Policy (Python) → aic_model (ROS 2 wrapper) → aic_controller (C++ impedance) → Gazebo (UR5e)
                                                                ↑ observations (camera, joint states, FT sensor)
```

**Inter-container communication:** ROS 2 via Zenoh (`rmw_zenoh_cpp`)

## Key Packages

| Package | Language | Purpose |
|---------|----------|---------|
| `aic_model` | Python | Lifecycle node that dynamically loads participant policies |
| `aic_example_policies` | Python | 7 reference implementations (WaveArm, CheatCode, RunACT, etc.) |
| `aic_engine` | C++ | Trial orchestrator & state machine |
| `aic_controller` | C++ | ros2_control impedance/joint controller plugin |
| `aic_adapter` | C++ | Sensor fusion (camera, joint states, FT sensor) |
| `aic_bringup` | Launch | Gazebo + UR5e startup, 50+ parameters |
| `aic_interfaces` | IDL | All ROS 2 msg/srv/action definitions |
| `aic_utils/aic_mujoco` | Python | MuJoCo alternative simulator integration |
| `aic_utils/lerobot_robot_aic` | Python | LeRobot driver (teleoperation, data recording) |

## Implementing a Policy

Policies live in `aic_example_policies/aic_example_policies/ros/` as reference. A policy must:

1. Inherit from `Policy` (`aic_model/aic_model/policy.py`)
2. Implement `insert_cable(task, get_observation, move_robot, send_feedback)`
3. Call `move_robot(MotionUpdate(...))` for Cartesian control or `JointMotionUpdate` for joint control

**Minimal example:** `aic_example_policies/aic_example_policies/ros/WaveArm.py`
**Ground-truth cheat:** `aic_example_policies/aic_example_policies/ros/CheatCode.py`
**LeRobot ACT policy:** `aic_example_policies/aic_example_policies/ros/RunACT.py`

## Key Interfaces

- `MotionUpdate.msg` — Cartesian impedance command (pose, 6×6 stiffness/damping matrices, wrench)
- `JointMotionUpdate.msg` — Joint trajectory command
- `InsertCable.action` — Main task action (Goal: task info; Result: success; Feedback: progress)
- `Observation.msg` — Camera images + joint states + FT sensor + TF transforms

Full reference: `docs/aic_interfaces.md`

## Environment & Dependencies

- **ROS 2 Kilted Kaiju** — official evaluation distribution
- **Pixi** (`pixi.toml`) — manages Python deps including LeRobot v0.4.3, MuJoCo v3.5.0
- **vcstool** (`aic.repos`) — pulls external ROS packages (UR driver, gz_ros2_control fork, etc.)
- The colcon workspace root is `~/ws_aic/`; this repo lives at `~/ws_aic/src/aic/`

## Documentation

All challenge documentation is in `docs/` (18 files). Key files:
- `docs/getting_started.md` — environment setup
- `docs/policy.md` — policy implementation tutorial
- `docs/aic_interfaces.md` — complete topic/service/action reference
- `docs/aic_controller.md` — impedance controller design
- `docs/submission_guidelines.md` — packaging & submission


## Current Progress（截至 2026-03-12）

### CheatCode Evaluation — 3/3 全部成功 ✅

| 任務 | 目標 | 初始 XY Error | 結果 |
|------|------|--------------|------|
| Task 1 | SFP → nic_card_mount_0 / sfp_port_0 | -11.7mm, -1.8mm | ✅ True |
| Task 2 | SFP → nic_card_mount_1 / sfp_port_0 | -11.6mm, +38.3mm | ✅ True |
| Task 3 | SC  → sc_port_1 / sc_port_base     | -104mm, +28mm   | ✅ True |

### TODO（按優先順序）

- [ ] 用 CheatCode 錄 100+ 次 ROS bag（訓練資料）
- [ ] 設定 RunPod 雲端訓練環境
- [ ] 用 lerobot-train 訓練 ACT 模型
- [ ] 把 model weights 塞回 Docker image → 本地測試
- [ ] 拿 Intrinsic auth token，提交到 registry

---

## Working Docker Workflow

> ⚠️ **不要用 docker-compose 跑 model service**
> Zenoh 跨 container 連不起來。正確做法是在 eval container 內直接跑 policy。

```bash
# 1. 啟動 eval container（背景）
cd ~/ws_aic/src/aic
sudo docker compose -f docker/docker-compose.yaml up -d eval

# 2. 在 eval container 裡跑 policy
sudo docker exec -it aic-eval-1 bash -c   "source /ws_aic/install/setup.bash &&    RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run aic_model aic_model    --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.CheatCode"

# 3. 看評分
sudo docker logs aic-eval-1 2>&1 | grep -E "score|trial|success|insert|result" | tail -20

# 4. 開 RViz 視覺化（另開 terminal）
sudo docker run --rm   --network container:aic-eval-1   --gpus all   --entrypoint bash   -e DISPLAY=:0   -e WAYLAND_DISPLAY=wayland-0   -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir   -e RMW_IMPLEMENTATION=rmw_zenoh_cpp   -e ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=false'   --volume /tmp/.X11-unix:/tmp/.X11-unix   --volume /mnt/wslg:/mnt/wslg   ghcr.io/intrinsic-dev/aic/aic_eval:latest   -c "source /ws_aic/install/setup.bash &&       ros2 run rviz2 rviz2 -d /ws_aic/install/share/aic_bringup/rviz/aic.rviz"
```

---

## Training Pipeline

```
CheatCode（自動示範）→ lerobot-record（收 ROS bag）→ lerobot-train（訓練 ACT）→ RunACT（推理）
```

**錄資料指令：**
```bash
cd ~/ws_aic/src/aic
pixi run lerobot-record   --robot.type=aic_controller --robot.id=aic   --teleop.type=aic_keyboard_ee --teleop.id=aic   --dataset.repo_id=<your-hf-username>/aic_dataset   --dataset.single_task="cable insertion"   --dataset.push_to_hub=false
```

**訓練指令（RunPod 上跑）：**
```bash
pixi run lerobot-train   --dataset.repo_id=<your-hf-username>/aic_dataset   --policy.type=act   --output_dir=outputs/train/act_aic   --policy.device=cuda
```

---

## RunACT Technical Details

**State Space（26維）：**
- TCP Position (3): x, y, z
- TCP Orientation (4): quaternion
- TCP Linear Velocity (3)
- TCP Angular Velocity (3)
- TCP Error (6)
- Joint Positions (7)

**Action Space（7維）：**
- Cartesian Twist: linear xyz(3) + angular xyz(3) + 1 未使用
- 送速度指令（MODE_VELOCITY），約 4Hz，在 base_link frame

**官方預訓練模型：** `grkw/aic_act_policy`（HuggingFace，可直接試跑）

---

## Known Issues & Gotchas

| 問題 | 原因 | 解法 |
|------|------|------|
| Zenoh 跨 container 連不起來 | docker-compose model service 網路隔離 | 改用 `docker exec` 在 eval container 內跑 policy |
| RViz 在 `docker exec` 裡沒 display | socket 沒掛進 container | 用 `docker run --network container:aic-eval-1` 另開 |
| `pixi install` WSL 本地失敗 | pixi-build-ros backend 問題 | 改用純 Docker 工作流 |
| container 重啟後 DISPLAY 消失 | env var 沒傳進去 | `sudo docker exec -it -e DISPLAY=$DISPLAY aic-eval-1 bash` |
| RunACT 跑 30 秒就 return True | 設計如此 | 訓練好的模型才能真正判斷插沒插成功 |

---

## CheatCode 技術觀察

- **兩階段設計：** Approach（pfrac 0→1，純軌跡追蹤）→ Insertion（z_offset 下壓，PI 校正 XY）
- **PI integrator clamp = ±0.05**（Task 2/3 大偏差任務會被 clamp，但仍成功）
- Task 3 SC 連接器初始 X 偏差 104mm，approach 路徑最長，容錯空間最小
- 插線穩定等待約 90–120 秒（z_offset = -0.015 之後）
- 各任務初始偏差是固定幾何偏差，不是隨機的

---

## Environment

- **GPU:** NVIDIA GeForce 6GB VRAM, CUDA 12.0, WSL2 GPU passthrough
- **Deadline:** 資格賽截止 **2026/05/15**
- **雲端訓練:** RunPod（本地 VRAM 不足）
- **視覺化:** RViz（WSL）/ Foxglove Studio（Windows，尚未安裝）
- **隊友共享:** 共享 git repo，隊友自行 `docker compose build model`
