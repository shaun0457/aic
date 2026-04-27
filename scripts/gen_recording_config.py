#!/usr/bin/env python3
"""Generate a recording config YAML with N repetitions of each of the 3 trial types.

Usage:
    python3 scripts/gen_recording_config.py --n_repeats 37 --out recording_config.yaml

This produces a YAML with 3 × n_repeats = 111 trials cycling through:
  - trial type A: SFP → nic_card_mount_0 / sfp_port_0
  - trial type B: SFP → nic_card_mount_1 / sfp_port_0
  - trial type C: SC  → sc_port_1 / sc_port_base
"""

import argparse
import copy
import yaml
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
SAMPLE_CONFIG = SCRIPT_DIR.parent / "aic_engine" / "config" / "sample_config.yaml"


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--n_repeats", type=int, default=37,
                   help="Number of times to repeat the 3 base trials (default 37 → 111 total)")
    p.add_argument("--out", type=Path, default=SCRIPT_DIR.parent / "recording_config.yaml")
    return p.parse_args()


def main():
    args = parse_args()

    with open(SAMPLE_CONFIG) as f:
        base = yaml.safe_load(f)

    base_trials = base["trials"]  # dict: trial_1, trial_2, trial_3
    trial_templates = list(base_trials.values())  # [trial_A, trial_B, trial_C]

    new_trials = {}
    idx = 1
    for _ in range(args.n_repeats):
        for template in trial_templates:
            key = f"trial_{idx}"
            new_trials[key] = copy.deepcopy(template)
            idx += 1

    out_config = copy.deepcopy(base)
    out_config["trials"] = new_trials

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w") as f:
        yaml.dump(out_config, f, default_flow_style=False, sort_keys=False)

    total = len(new_trials)
    print(f"Generated {total} trials → {args.out}")
    print(f"  (3 trial types × {args.n_repeats} repeats)")


if __name__ == "__main__":
    main()
