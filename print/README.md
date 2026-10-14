# Everything to print

These are the parts of the robot as it walks in the videos: the v1.3 body (r3 tub), the Hip22 hip
block, the v1.3 legs and the R80 rocker shoes. Every STL is already in its print orientation.

![All the printed parts](../docs/print_parts.jpg)

| File | Qty | Notes |
| ---- | --- | ----- |
| `00_servo_fit_coupon.stl` | 1 | Print this first: an SG90 must slide into it snugly. If it doesn't, fix your printer's tolerances before printing the rest. |
| `body/01_trunk_tub.stl` | 1 | The body. Floor down, no supports. A sliced plate is in `01_trunk_tub.3mf`. |
| `body/02_trunk_lid.stl` | 1 | The lid. |
| `body/15_battery_cassette.stl` | 1 | Holds the flat LiPo under the body and screws shut from below: no glue. |
| `hips/03_hip_block.stl` | 1 | Both hip-roll servos live here. |
| `hips/04_hip_bracket_L.stl`, `04_hip_bracket_R.stl` | 1 each | |
| `legs/05_thigh_L.stl`, `05_thigh_R.stl` | 1 each | |
| `legs/06_shin_L.stl`, `06_shin_R.stl` | 1 each | |
| `legs/07_foot_L.stl`, `07_foot_R.stl` | 1 each | The flat feet. The shoes slide over them. |
| `shoes/01_rocker_shoe.stl` | 2 | Curved sole, 80 mm radius. Prints standing on its flat front edge. |
| `shoes/02_rear_gate.stl` | 2 | The stop that clicks in behind the foot. |
| `shoes/03_taper_key.stl` | 2 | The wedge pin that locks the shoe. |

`shoes/shoes_plate_x2.3mf` has both shoes with their stops and keys already placed and oriented, and
`shoes/shoe_fit_test.3mf` is a small test of the shoe's slide and key: print it first.

## Settings

PLA, 0.2 mm layers (0.16 mm for the shoes), 3–4 walls, 20 % infill for the body and legs. Nothing
needs supports except the shoes: they stand on their edge, so give them a brim (8 mm) and supports
from the plate only, under the guide rails. See `shoes/print-orientation.png`.

## Screws

M2 screws hold the body, the boards and the battery cassette. The face (OLED) takes **M2×4**:
longer ones come out through the front. The horns use the small screws that come with the SG90s.

## Editing

STEP files for every part are in [`../cad/v1.3/step`](../cad/v1.3/step), and the full FreeCAD assembly is
[`../cad/v1.3/Duckbert_v1_3.FCStd`](../cad/v1.3). A stiffer hip block that bolts on the same way is in
[`../cad/v1.3/hip_block_r2`](../cad/v1.3/hip_block_r2). It hasn't been printed yet.
