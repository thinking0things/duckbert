# Calibration

The firmware knows what angle every joint should be at. It knows nothing about where *your*
servo horn ended up.

An SG90 horn sits on a toothed shaft, so it only goes on in steps: one tooth is about 18°. Push it on
as straight as you can and it can still be up to half a tooth off, about 9°. Six servos, six random
errors. On a robot whose foot is 41 mm wide, a few degrees at the hip moves the weight off the foot.

So before he walks, every joint gets a **trim**: a correction, in degrees, saved on the board.

## With the guided page

1. Build the robot, flash the firmware, plug in USB and run `python3 control/duckbert_web.py`.
2. Open `http://localhost:8080/calibrate`.
3. Press **Wake up** and hold him in the air: the feet must not touch anything.
4. Follow the four steps. In each one the robot takes a pose, and you nudge two joints until they
   match the drawing:
   - **Hip roll**, from the front: both brackets under the body level.
   - **Thighs**, from the side: hip screw to knee screw at 90° to the body.
   - **Shins**, from the side: knee screw to foot bolts at 90° to the body.
   - **Check**: legs fully straight, both the same.
5. Press **Save to the board**. Without it, the trims are gone at the next restart.

Measure against the body with a set square, not against the table. Not sure which way a joint
goes? Press +1 and look. If it gets worse, press −1 twice.

## If a trim is more than about 15°

The horn is a whole tooth off. Take it off, turn it one tooth and put it back, then calibrate that joint
again. A big trim works, but it eats into the joint's travel.

## After a fall

Falls can make a horn slip. If he suddenly walks worse, run the check step again before anything
else.
