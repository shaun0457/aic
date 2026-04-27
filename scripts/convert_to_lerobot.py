#!/usr/bin/env python3
"""Convert recorded episodes (numpy+jpg) to LeRobot dataset format.

Usage:
    pixi run python scripts/convert_to_lerobot.py \
        --src ./aic_dataset \
        --repo_id <your-hf-username>/aic_cheatcode_demos \
        [--push_to_hub]

Input structure (from CheatCodeRecorder):
    src/episode_000000/
        frames.npz      -- states (T,26), actions (T,7)
        left/TTTTT.jpg
        center/TTTTT.jpg
        right/TTTTT.jpg
        meta.json

Output: LeRobotDataset at ~/.cache/huggingface/lerobot/<repo_id>/
"""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--src", required=True, type=Path)
    p.add_argument("--repo_id", required=True)
    p.add_argument("--push_to_hub", action="store_true")
    return p.parse_args()


def main():
    args = parse_args()
    src = args.src

    episodes = sorted(src.glob("episode_*"))
    if not episodes:
        print(f"No episodes found in {src}")
        return

    print(f"Found {len(episodes)} episodes in {src}")

    # Import here so the script fails loudly if lerobot is missing
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    # Camera: 1152×1024 px scaled by 0.25 → W=288, H=256. Shape is (H, W, C).
    IMG_SHAPE = (256, 288, 3)

    features = {
        "observation.state": {"dtype": "float32", "shape": (26,), "names": None},
        "action": {"dtype": "float32", "shape": (7,), "names": None},
        "observation.images.left_camera": {
            "dtype": "image",
            "shape": IMG_SHAPE,
            "names": ["height", "width", "channel"],
        },
        "observation.images.center_camera": {
            "dtype": "image",
            "shape": IMG_SHAPE,
            "names": ["height", "width", "channel"],
        },
        "observation.images.right_camera": {
            "dtype": "image",
            "shape": IMG_SHAPE,
            "names": ["height", "width", "channel"],
        },
    }

    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=20,
        features=features,
        robot_type="ur5e",
    )

    for ep_dir in episodes:
        meta_path = ep_dir / "meta.json"
        if not meta_path.exists():
            print(f"  Skipping {ep_dir.name}: no meta.json")
            continue

        with open(meta_path) as f:
            meta = json.load(f)

        frames = np.load(ep_dir / "frames.npz")
        states = frames["states"]    # (T, 26)
        actions = frames["actions"]  # (T, 7)
        T = len(states)

        print(f"  {ep_dir.name}: {T} frames  task={meta.get('task','?')}  success={meta.get('success')}")

        for t in range(T):
            frame = {
                "observation.state": states[t],
                "action": actions[t],
            }
            for cam in ("left", "center", "right"):
                img_path = ep_dir / cam / f"{t:05d}.jpg"
                if img_path.exists():
                    frame[f"observation.images.{cam}_camera"] = np.array(Image.open(img_path))
                else:
                    frame[f"observation.images.{cam}_camera"] = np.zeros(IMG_SHAPE, dtype=np.uint8)
            dataset.add_frame(frame)

        dataset.save_episode()

    print(f"\nDataset saved to {dataset.root}")

    if args.push_to_hub:
        print(f"Pushing to hub: {args.repo_id}")
        dataset.push_to_hub()


if __name__ == "__main__":
    main()
