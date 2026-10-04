"""Build the standalone public CartPole MPC lab into index.html."""
from __future__ import annotations
import argparse, json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
ASSETS = ROOT / "assets"

def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")

def build() -> str:
    shell = read(SRC / "shell.html")
    actor = json.loads(read(ASSETS / "ppo_actor.json"))
    residual = json.loads(read(ASSETS / "residual_model.json"))
    parts = {
        "STYLE": read(SRC / "styles.css"),
        "CORE": "\n".join([
            read(SRC / "bam_params.js"),
            read(SRC / "plant.js"),
            read(ROOT / "vendor/quadprog/quadprog.js"),
            read(SRC / "qp.js"),
            read(SRC / "engine.js"),
        ]),
        "ACTOR": "const CONTROL_LAB_ACTOR = " + json.dumps(actor, separators=(",", ":")) + ";",
        "RESIDUAL": "const CONTROL_LAB_RESIDUAL = " + json.dumps(residual, separators=(",", ":")) + ";",
        "TOPICS": read(SRC / "topics.js"),
        "APP": read(SRC / "app.js"),
    }
    for key, value in parts.items():
        token = "/* " + key + " */"
        if shell.count(token) != 1:
            raise ValueError("expected exactly one " + token)
        shell = shell.replace(token, value)
    return shell

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    out = ROOT / "index.html"
    data = build()
    if args.check:
        if not out.exists() or read(out) != data:
            raise SystemExit("index.html is stale; run python3 scripts/build.py")
        print("index.html: up to date")
    else:
        out.write_text(data, encoding="utf-8")
        print(f"index.html: {out.stat().st_size:,} bytes")

if __name__ == "__main__":
    main()
