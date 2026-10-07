"""Rebuild MuJoCo/URDF assets directly from wheeled-legged_RL's source USD."""
from __future__ import annotations

from pathlib import Path
import subprocess
import sys


HERE = Path(__file__).resolve().parent
SCRIPTS = (
    "usd_dump_full.py",
    "gen_urdf_mjcf.py",
    "gen_mjcf2.py",
    "gen_model.py",
)


def main():
    for name in SCRIPTS:
        path = HERE / "tools" / name
        print(f"\n=== {name} ===", flush=True)
        subprocess.run([sys.executable, str(path)], cwd=HERE, check=True)
    print("\nModels rebuilt from the source USD.")


if __name__ == "__main__":
    main()
