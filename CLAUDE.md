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


## Current Progress（截至 2026-05-02）

### CheatCode Evaluation — 3/3 全部成功 ✅

| 任務 | 目標 | 初始 XY Error | 結果 |
|------|------|--------------|------|
| Task 1 | SFP → nic_card_mount_0 / sfp_port_0 | -11.7mm, -1.8mm | ✅ True |
| Task 2 | SFP → nic_card_mount_1 / sfp_port_0 | -11.6mm, +38.3mm | ✅ True |
| Task 3 | SC  → sc_port_1 / sc_port_base     | -104mm, +28mm   | ✅ True |

### 訓練資料集 ✅

- **111 個 episodes** 已錄製（CheatCodeRecorder）
- 原始資料備份：`~/ws_aic/src/aic/aic_dataset_partial/`（8.7GB，含 tar）
- HuggingFace dataset：`shaun0457/aic_cheatcode_demos`（private，tar 格式）
- lerobot 格式：`~/.cache/huggingface/lerobot/shaun0457/aic_cheatcode_demos/`（7.3GB，110 episodes）
- 轉換指令：`python3 scripts/convert_to_lerobot.py --src aic_dataset_partial --repo_id shaun0457/aic_cheatcode_demos`

### ACT 模型訓練 ✅

- **HuggingFace model：** `shaun0457/act_aic`（100K steps，final loss ~0.065）
- 訓練在 GCP VM（Tesla T4, 15GB VRAM），約 6.5 小時
- lerobot-train 指令（不用 pixi，直接用系統 lerobot）：
```bash
lerobot-train \
  --dataset.repo_id=shaun0457/aic_cheatcode_demos \
  --policy.type=act \
  --policy.repo_id=shaun0457/act_aic \
  --output_dir=outputs/train/act_aic \
  --policy.device=cuda
```

### RunACT Docker 測試 ✅

- Image：`my-solution:v1`（本地 build）
- 測試結果：**總分 107.96 / ~300**（Trial 1: 39.8, Trial 2: 42.8, Trial 3: 25.4）
- 跑法見下方 Working Docker Workflow

### TODO（按優先順序）

- [x] 用 CheatCode 錄 100+ 次 ROS bag ✅
- [x] 轉換為 lerobot 格式 ✅
- [x] 訓練 ACT 模型（GCP VM） ✅
- [x] RunACT Docker image build + 測試通過 ✅
- [ ] 繼續優化分數（更長推理時間、更多訓練）
- [ ] 拿 Intrinsic auth token，提交到 registry（截止 2026/05/15）

---

## Working Docker Workflow

> ⚠️ **不要用 docker-compose 跑 model service**
> Zenoh 跨 container 連不起來。正確做法：eval container 用 docker-compose，model container 用 `--network container:aic-eval-1` 共享網路。

```bash
# 1. 啟動 eval container
cd ~/ws_aic/src/aic
sudo docker compose -f docker/docker-compose.yaml up -d eval

# 2a. 跑 CheatCode（在 eval container 內，不需要 torch）
sudo docker exec -it aic-eval-1 bash -c \
  "source /ws_aic/install/setup.bash && \
   RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run aic_model aic_model \
   --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.CheatCode"

# 2b. 跑 RunACT（用自訂 image，共享 eval 網路）
sudo docker rm -f aic-model-1 2>/dev/null
sudo docker run --rm -d \
  --name aic-model-1 \
  --network container:aic-eval-1 \
  --gpus all \
  -e AIC_ROUTER_ADDR=localhost:7447 \
  my-solution:v1 \
  --ros-args -p use_sim_time:=true -p policy:=aic_example_policies.ros.RunACT

# 3. 看評分
sudo docker logs aic-eval-1 2>&1 | grep -E "score|trial|success|result" | tail -20
sudo docker logs aic-model-1 2>&1 | tail -20

# 4. Build model image
docker build -f docker/aic_model/Dockerfile -t my-solution:v1 .
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

**訓練指令（GCP VM 或任何有 GPU 的機器，直接用系統 lerobot）：**
```bash
lerobot-train \
  --dataset.repo_id=shaun0457/aic_cheatcode_demos \
  --policy.type=act \
  --policy.repo_id=shaun0457/act_aic \
  --output_dir=outputs/train/act_aic \
  --policy.device=cuda
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
| Zenoh 跨 container 連不起來 | docker-compose model service 網路隔離 | RunACT 用 `docker run --network container:aic-eval-1`（CheatCode 仍用 docker exec）|
| RViz 在 `docker exec` 裡沒 display | socket 沒掛進 container | 用 `docker run --network container:aic-eval-1` 另開 |
| `pixi install --locked` Docker build 失敗 | lock file 在 macOS 生成，osx-arm64 依賴衝突 | 改用 `--frozen`；build 後 cp source 覆蓋 site-packages |
| pixi site-packages 是舊版本 | pixi install 用 conda 快取，不用本地 source | Dockerfile 加 `cp -r aic_example_policies/. .pixi/envs/.../site-packages/aic_example_policies/` |
| RunACT module import 慢 15 秒 → GetState timeout | `from aic_model.policy import Policy` 觸發大量 ROS message import | RunACT 不繼承 Policy，改用輕量 `_LightPolicy` stub，所有 heavy import 移入 `__init__` |
| RunACT 網路請求失敗（isolated network）| `snapshot_download` + ResNet18 backbone 嘗試連 HuggingFace | Dockerfile 預下載所有 weights；hardcode 快取路徑；`ENV HF_HUB_OFFLINE=1` |
| container 重啟後 DISPLAY 消失 | env var 沒傳進去 | `sudo docker exec -it -e DISPLAY=$DISPLAY aic-eval-1 bash` |

---

## CheatCode 技術觀察

- **兩階段設計：** Approach（pfrac 0→1，純軌跡追蹤）→ Insertion（z_offset 下壓，PI 校正 XY）
- **PI integrator clamp = ±0.05**（Task 2/3 大偏差任務會被 clamp，但仍成功）
- Task 3 SC 連接器初始 X 偏差 104mm，approach 路徑最長，容錯空間最小
- 插線穩定等待約 90–120 秒（z_offset = -0.015 之後）
- 各任務初始偏差是固定幾何偏差，不是隨機的

---

## Environment

- **本地 GPU:** NVIDIA GeForce 6GB VRAM, CUDA 12.0, WSL2 GPU passthrough（VRAM 不足訓練）
- **GCP VM GPU:** Tesla T4 15GB VRAM, CUDA 12.8（用於 ACT 訓練 + Docker build + 測試）
- **Deadline:** 資格賽截止 **2026/05/15**
- **視覺化:** RViz（WSL）/ Foxglove Studio（Windows，尚未安裝）
- **隊友共享:** 共享 git repo，隊友自行 `docker compose build model`

### 遷移 GCP → 本地

GCP VM 太貴，需搬遷：
```bash
# 儲存 Docker image 為 tar（在 GCP VM 上）
docker save my-solution:v1 | gzip > my-solution-v1.tar.gz

# 傳回本地（在本地執行）
gcloud compute scp <vm-name>:~/my-solution-v1.tar.gz .

# 本地載入
docker load < my-solution-v1.tar.gz

# 或 push 到 Docker Hub（在 GCP VM 上）
docker tag my-solution:v1 <dockerhub-user>/my-solution:v1
docker push <dockerhub-user>/my-solution:v1
```
