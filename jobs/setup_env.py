"""Set up (or update) the project on the Ray node in ~/Desktop/IROS26: the code from this upload, and
everything else inside that folder too (uv, Python 3.12 and uv's cache in .uv/, the environments in
.venv/ and simulator/.venv/), both environments synced, then a smoke test: the simulator's own tests,
and the camera tests, which run the simulator rendering headless on the GPU (MUJOCO_GL=egl).

    uv run python jobs/ray_job.py setup_env logs/jobs/setup_env.json

Run it again after changing the code: it syncs the code (keeping the environments and the rig data)."""
import base64
import json
import os
import subprocess
from pathlib import Path

TREE = ["pyproject.toml", "uv.lock", "myaloha", "simulator", "tests", "config", "panel",
        "thirdparty/lerobot_trossen", "jobs"]
HOME = Path("~/Desktop/IROS26").expanduser()
TOOLS = HOME / ".uv"
KEEP = [".venv", ".uv", "logs", "calib", "simulator/.venv"]      # not replaced by an update of the code

env = {k: v for k, v in os.environ.items() if k not in ("VIRTUAL_ENV", "PYTHONPATH", "PYTHONHOME", "CONDA_PREFIX")}
env.update(UV_INSTALL_DIR=str(TOOLS / "bin"), UV_CACHE_DIR=str(TOOLS / "cache"),
           UV_PYTHON_INSTALL_DIR=str(TOOLS / "python"), INSTALLER_NO_MODIFY_PATH="1", MUJOCO_GL="egl",
           PATH=f"{TOOLS / 'bin'}:{env.get('PATH', '')}")
steps = []


def step(name, cmd, cwd=HOME, timeout=3600):
    r = subprocess.run(cmd, shell=True, cwd=cwd, env=env, capture_output=True, text=True, timeout=timeout)
    steps.append({"step": name, "rc": r.returncode, "out": (r.stdout + r.stderr)[-2500:]})
    print(f"== {name}: rc {r.returncode}", flush=True)
    return r.returncode == 0


HOME.mkdir(parents=True, exist_ok=True)
ok = step("code", "rsync -a --delete " + " ".join(f"--exclude /{k}" for k in KEEP) + f" ./ {HOME}/", cwd=".")
ok = ok and (step("uv (present)", "uv --version") or step("uv", "curl -LsSf https://astral.sh/uv/install.sh | sh"))
ok = ok and step("backend environment", "uv sync")
ok = ok and step("simulator environment", "uv sync", cwd=HOME / "simulator")
ok = ok and step("simulator tests", "uv run pytest -q", cwd=HOME / "simulator")
ok = ok and step("camera tests (the simulator, rendering on the GPU)", "uv run pytest -q tests/test_cameras.py")
print("RESULT_BEGIN\n" + base64.b64encode(json.dumps(steps).encode()).decode() + "\nRESULT_END", flush=True)
