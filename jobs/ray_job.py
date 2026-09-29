"""Run a job from jobs/ on the Ray cluster (ray.sqrtwo.com: the Jobs API over HTTPS only) and save what it
returns. A job is a script jobs/NAME.py that lists the repo files it needs in FILES (uploaded flat, next
to it) and TREE (files and folders uploaded at their paths in the repo; read here without importing
it: the cluster has Ray, this environment doesn't), and prints its result, base64-encoded, between the
lines RESULT_BEGIN and RESULT_END.

    uv run python jobs/ray_job.py NAME OUT [--address URL]

Uses the Ray CLI through uvx, pinned to the cluster's Ray version."""
import argparse
import ast
import base64
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ADDRESS = "https://ray.sqrtwo.com"
RAY = ["uvx", "--python", "3.12", "--from", "ray[default]==2.58.0", "ray"]


def listed(script: Path, name: str) -> list[str]:
    for node in ast.parse(script.read_text()).body:
        target = node.target if isinstance(node, ast.AnnAssign) else getattr(node, "targets", [None])[0]
        if getattr(target, "id", None) == name and node.value is not None:
            return ast.literal_eval(node.value)
    return []


SKIP = shutil.ignore_patterns(".venv", "__pycache__", ".git", ".pytest_cache", "*.pyc", ".DS_Store")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", help="the job: jobs/NAME.py")
    ap.add_argument("out", help="where to save its result")
    ap.add_argument("--address", default=ADDRESS)
    args = ap.parse_args()
    script = ROOT / "jobs" / f"{args.name}.py"
    (ROOT / "logs" / "jobs").mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=ROOT / "logs" / "jobs") as tmp:   # uploaded: the script and its FILES
        shutil.copy(script, tmp)
        for f in listed(script, "FILES"):
            shutil.copy(ROOT / f, tmp)
        for f in listed(script, "TREE"):
            (Path(tmp) / f).parent.mkdir(parents=True, exist_ok=True)
            if (ROOT / f).is_dir():
                shutil.copytree(ROOT / f, Path(tmp) / f, ignore=SKIP)
            else:
                shutil.copy(ROOT / f, Path(tmp) / f)
        run = subprocess.run([*RAY, "job", "submit", "--address", args.address, "--working-dir", tmp, "--",
                              "python", script.name], capture_output=True, text=True)
    log = run.stdout + run.stderr
    m = re.search(r"RESULT_BEGIN\s*\n(.*?)\n\s*RESULT_END", log, re.S)
    if run.returncode or not m:
        sys.exit(f"the job failed or returned nothing:\n{log[-3000:]}")
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_bytes(base64.b64decode("".join(m.group(1).split())))
    summary = [line for line in log.splitlines() if not re.fullmatch(r"[A-Za-z0-9+/=]{100,}", line.strip())]
    print("\n".join(summary[-8:]))
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
