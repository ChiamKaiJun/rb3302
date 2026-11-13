"""python3 -m stage1_sim --start S --bearing E --goals T,N,H,A"""

from __future__ import annotations

import argparse
from pathlib import Path

from stage1_sim.session import format_report, run_stage1, save_preview


def main() -> None:
    parser = argparse.ArgumentParser(description="Stage 1 checks: grid input, map/spawn view, A* to the first of four goals.")
    parser.add_argument("--start", required=True, help="Spawn letter A-X")
    parser.add_argument("--bearing", required=True, help="Spawn heading: N NE E SE S SW W NW")
    parser.add_argument("--goals", required=True, help="Exactly four letters, comma-separated. Repeats allowed.")
    parser.add_argument("--out", default="stage1_preview.png", help="Preview image path")
    args = parser.parse_args()
    goals = [part.strip() for part in args.goals.split(",") if part.strip()]
    result = run_stage1(args.start, args.bearing, goals)
    out = Path(args.out)
    save_preview(result, str(out))
    print(format_report(result))
    print(f"preview {out.resolve()}")
    if not result.ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
