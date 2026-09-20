"""Run the Duckbert v1.1 Claude walking example in the native MuJoCo viewer."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "dist" / "assets" / "robot.xml"
GAIT_PATH = ROOT / "dist" / "assets" / "gaits.json"


def gait_command(params, phase, turn):
    """Return six joint targets using the same periodic gait as the browser demo."""
    period, hip, knee = params[:3]
    s = math.sin(2 * math.pi * phase / period)
    c = math.cos(2 * math.pi * phase / period)
    s2 = math.sin(4 * math.pi * phase / period)
    c2 = math.cos(4 * math.pi * phase / period)
    swing_hip = params[3] * s + params[4] * c
    even_hip = params[5] * s2 + params[6] * c2
    swing_knee = params[7] * s + params[8] * c
    even_knee = params[9] * s2 + params[10] * c2
    lean = params[11] * s + params[12] * c
    targets = [lean + params[13], hip + swing_hip + even_hip, knee + swing_knee + even_knee,
               lean - params[13], hip - swing_hip + even_hip, knee - swing_knee + even_knee]
    midpoint = (targets[1] + targets[4]) / 2
    gain = 0.10
    targets[1] = midpoint + (targets[1] - midpoint) * (1 - gain * turn)
    targets[4] = midpoint + (targets[4] - midpoint) * (1 + gain * turn)
    return targets


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=12.0, help="Run time before closing (default: 12)")
    parser.add_argument("--turn", type=float, default=0.0, help="Steering from -1 (left) to 1 (right)")
    args = parser.parse_args()
    if not -1 <= args.turn <= 1:
        parser.error("--turn must be between -1 and 1")

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    gait = json.loads(GAIT_PATH.read_text())[0]
    period = gait["params"][0]
    control_period = 0.02  # 50 Hz, matching the browser controller.
    next_control = 0.0
    phase = 0.0
    start = time.monotonic()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and time.monotonic() - start < args.seconds:
            if data.time >= next_control:
                data.ctrl[:] = gait_command(gait["params"], phase, args.turn)
                next_control += control_period
            mujoco.mj_step(model, data)
            phase += model.opt.timestep
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep * 0.5))


if __name__ == "__main__":
    main()
