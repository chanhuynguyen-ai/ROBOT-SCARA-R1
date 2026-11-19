"""Repository entry point for the R1 P7 research simulation."""
from __future__ import annotations
import argparse
from pathlib import Path
import subprocess
import sys

REPOSITORY = Path(__file__).resolve().parents[1]
MODES = {"verify": "Verify", "audit": "Audit", "scene": "SceneCheck",
         "native": "NativeCheck", "view": "ViewMuJoCo", "compare": "ViewSafe", "mesh": "MeshAudit"}

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=MODES, default="verify")
    parser.add_argument("--output", type=Path, default=REPOSITORY / "artifacts")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--speed", type=float, default=0.8)
    parser.add_argument("--no-loop", action="store_true")
    args = parser.parse_args(argv)
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, "-B", str(REPOSITORY / "simulation/p7/tools/r1_p7_windows.py"),
               "--ProjectRoot", str(output), "--FreezeDir", str(REPOSITORY / "reference/v13"),
               "--Mode", MODES[args.mode]]
    if args.mode == "view":
        command.extend(["--StartFrame", str(args.start_frame), "--Speed", str(args.speed)])
        if args.no_loop:
            command.append("--NoLoop")
    return subprocess.run(command, cwd=REPOSITORY, check=False).returncode

if __name__ == "__main__":
    raise SystemExit(main())
