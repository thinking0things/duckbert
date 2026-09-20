# Run Duckbert in MuJoCo

This folder explains how to run the same Duckbert v1.1 Claude model locally with the official
Python MuJoCo bindings. The XML model and STL files are kept in `../dist/assets/`, so the native
viewer and the browser use the same geometry and physics asset.

## Requirements

- Python 3.10 or newer
- MuJoCo Python bindings 3.x
- A desktop session with an OpenGL-capable display for the interactive viewer

Create an environment and install the dependency:

```sh
cd headless-lab
python3 -m venv .venv-mujoco
. .venv-mujoco/bin/activate
python -m pip install --upgrade pip
python -m pip install -r mujoco/requirements.txt
```

## Open the model

The shortest path opens MuJoCo's native interactive viewer:

```sh
python -m mujoco.viewer --mjcf dist/assets/robot.xml
```

Use the viewer's mouse controls to orbit and inspect the robot. The model includes the v1.1 Claude
body, LiPo cassette, inward-offset flat feet, blue servo hardware, black OLED screen and white eyes.
The visible mesh palette is a renderer choice; MuJoCo uses the RGBA values stored in the XML.

## Run the walking example

`run_viewer.py` launches a passive viewer and drives the six position actuators with the same
`claude_v11_walk` gait used by Duckbert. Close the viewer window to exit.

```sh
python mujoco/run_viewer.py
python mujoco/run_viewer.py --seconds 12 --turn 0.5
```

`--turn` accepts values from -1 (left) to 1 (right). Set it to zero for a straight walk.
The example is intentionally small and readable so it can be copied into another MuJoCo project.
It does not connect to hardware or send serial commands.

## Use the model from your own Python program

```python
import mujoco

model = mujoco.MjModel.from_xml_path("dist/assets/robot.xml")
data = mujoco.MjData(model)
for _ in range(1000):
    data.ctrl[:] = 0.0
    mujoco.mj_step(model, data)
```

The XML uses a relative `meshdir="meshes"`, so keep `robot.xml` beside the `meshes/` directory
when copying the asset elsewhere. If you copy only the XML, mesh loading will fail.

## Asset and controller notes

- `dist/assets/robot.xml` is the v1.1 Claude six-servo model.
- `dist/assets/meshes/` contains the 32 neutral assembly STL files.
- `dist/assets/gaits.json` contains the browser gait parameters.
- `dist/assets/provenance.json` records the CAD revision and mesh checksums.
- The browser uses a WASM port of MuJoCo; this local example uses the native Python bindings.

The CAD mass properties are estimates based on solid PLA density, the manifest's module masses,
and a wiring/fastener allowance. Recheck the real battery and printed infill before using the
model for engineering predictions.
