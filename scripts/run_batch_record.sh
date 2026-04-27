#!/usr/bin/env bash
# run_batch_record.sh — Record N episodes with CheatCodeRecorder (one container launch)
#
# Usage:
#   cd ~/ws_aic/src/aic
#   bash scripts/run_batch_record.sh [N_REPEATS] [OUTPUT_DIR]
#
#   N_REPEATS  : number of times to repeat the 3 base trials (default: 37 → 111 episodes)
#   OUTPUT_DIR : dataset path inside container (default: /tmp/aic_dataset)
#
# After recording, copy dataset out:
#   sudo docker cp aic-aic-eval-1:/tmp/aic_dataset ./aic_dataset
#
# Then convert to lerobot format:
#   pixi run python scripts/convert_to_lerobot.py \
#       --src ./aic_dataset \
#       --repo_id <your-hf-username>/aic_cheatcode_demos

set -euo pipefail

N_REPEATS=${1:-37}        # 37 × 3 = 111 episodes
OUTPUT_DIR=${2:-/tmp/aic_dataset}
CONTAINER=aic-aic-eval-1

TOTAL=$((N_REPEATS * 3))
echo "=== Batch Recording Setup ==="
echo "  Repeats: $N_REPEATS  →  Total episodes: $TOTAL"
echo "  Dataset: $CONTAINER:$OUTPUT_DIR"
echo ""

# Step 1: Generate recording_config.yaml with N_REPEATS × 3 trials
echo "[1/4] Generating recording_config.yaml ($TOTAL trials)..."
pixi run python scripts/gen_recording_config.py \
    --n_repeats "$N_REPEATS" \
    --out recording_config.yaml
echo "      Done: recording_config.yaml"

# Step 2: Build Docker image with CheatCodeRecorder
echo ""
echo "[2/4] Building aic_model Docker image..."
sudo docker build -f docker/aic_model/Dockerfile -t my-solution:v1 .
echo "      Done."

# Step 3: Start eval container with recording config
echo ""
echo "[3/4] Starting eval container (ground_truth:=true, $TOTAL trials)..."
sudo docker compose \
    -f docker/docker-compose.yaml \
    -f docker/docker-compose.record.yaml \
    up -d eval

echo "      Waiting 60s for Gazebo + engine to initialise..."
sleep 60

# Step 4: Run CheatCodeRecorder (handles all trials, records all episodes)
echo ""
echo "[4/4] Running CheatCodeRecorder for $TOTAL episodes..."
echo "      (This will take ~$((TOTAL * 2)) min. Engine resets between trials automatically.)"
echo ""

sudo docker exec -it "$CONTAINER" bash -c "
    source /ws_aic/install/setup.bash
    RECORD_OUTPUT_DIR=${OUTPUT_DIR} \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    ros2 run aic_model aic_model \
        --ros-args \
        -p use_sim_time:=true \
        -p policy:=aic_example_policies.ros.CheatCodeRecorder
"

echo ""
echo "=== Recording complete: $TOTAL episodes ==="
echo ""
echo "Copy dataset out:"
echo "  sudo docker cp ${CONTAINER}:${OUTPUT_DIR} ./aic_dataset"
echo ""
echo "Then convert to lerobot format:"
echo "  pixi run python scripts/convert_to_lerobot.py \\"
echo "      --src ./aic_dataset \\"
echo "      --repo_id <your-hf-username>/aic_cheatcode_demos \\"
echo "      --push_to_hub"
