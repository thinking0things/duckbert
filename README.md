# Duckbert

**[Open the simulation](https://thinking0things.github.io/duckbert/)**

A headless SG90 robot with one walking gait and directional controls in the browser.
MuJoCo simulates gravity, contacts and six servomotors; Three.js renders the CAD meshes.
This is a standalone simulation with no connection to the physical robot.

## Controls

- **W / ↑**: walk forward.
- **S / ↓**: walk backward by reversing the phase of the same gait.
- **A / ←** and **D / →**: turn left or right while walking.
- **Space**: pause the simulation.
- With a mouse or touchscreen, tap a direction to keep moving; use **Pause** to freeze the simulation.
- **Reset to centre**: restore the starting position, orientation and gait phase.
- Drag to orbit the camera; scroll or pinch to zoom.

Directions are relative to the robot. Left and right produce curved turns rather than sideways
steps. Pausing freezes simulation time; it is not a physical stopping controller.
The simulation also pauses when the page loses focus.

## Model and controller

The baseline model is `microduck9_none_feet_in.xml`: the headless configuration with inward-facing
feet used to train the `walk_feetin_mid` gait. Meshes are frozen from the compatible
`shell160_pre_tof` archive because the main CAD folder was subsequently modified.
This simulation uses that baseline, not the unfinished CAD v1.1/v1.2 revisions.

The periodic controller is ported from `gait9.py`, with position commands at 50 Hz, torque limits
and command slew limits. Steering varies the relative hip swing amplitude by up to 10%.
Backward movement reverses the same periodic trajectory. The controller does not directly
set the free body's position or orientation, or apply artificial steering forces.

The automated check runs 12 seconds of forward, left, right and backward movement in the WASM
engine, checking that the robot stays upright and turns in the expected direction. These are
nominal checks, not a stability guarantee for every sequence: prolonged turns, changes of direction
or numerical differences can cause falls. Backward movement may drift.

## Local development

Requires Node.js 22+ and Python 3 for the static server.

```sh
npm ci
npm run vendor
npm test
npm run serve
```

Open `http://127.0.0.1:8768`. The `dist/` folder includes all runtime dependencies and can also
be served by any static HTTP server. No API keys or backend are required.

## GitHub Pages

The `main` branch contains source code, tests, data and pinned dependencies. The `gh-pages`
branch contains only the contents of `dist/`, served at the project site's root.

```sh
git add .
git commit -m "Update simulation"
git push origin main
git subtree split --prefix dist -b pages-release
git push origin pages-release:gh-pages
git branch -D pages-release
```

In Settings → Pages, select the `gh-pages` branch and the `/` folder.
Imports and assets use relative paths compatible with the project URL.

## Provenance and third-party licences

- Physics: [Google DeepMind MuJoCo](https://github.com/google-deepmind/mujoco), official
  `@mujoco/mujoco` package **3.6.0**, Apache-2.0; licence in `dist/vendor/mujoco/LICENSE`.
- Rendering: [Three.js](https://github.com/mrdoob/three.js) **0.178.0**, MIT;
  licence in `dist/vendor/three/LICENSE`.
- SG90 geometry and controller: the author's local Microduck project. The CAD reconstruction
  references the kinematic proportions of [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck);
  upstream meshes are not included in this repository.

Model provenance is recorded in `dist/assets/provenance.json`.
