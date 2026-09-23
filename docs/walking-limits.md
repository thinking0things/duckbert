# What the geometry caps, and two ways past it

Measured on the fitted model (see the main README's [Model and controller](../README.md#model-and-controller)
section for the fitting process): the centre of mass sits **99 mm** off the ground, the sole is
**41 mm** wide, and the robot has **no ankles**.

## Why that caps the walk

Without an ankle, rotating a hip rolls the trunk and the sole plate together — one rigid piece.
The robot can't shift its weight *inside* the foot by rolling the ankle underneath a level trunk;
the only motion available tips the whole foot onto its edge. At 11° of hip roll (its usable
range), that motion moves the centre of mass by only **2.7 mm**. Everything past that just tips
further over the edge instead of correcting anything.

Crouching gives a little of that back: at 40° of knee bend the centre of mass drops from 99 mm to
83 mm, and time-to-fall in simulation goes up by about 9%. Two crouched gaits were searched
anyway — a shallow 25° crouch and a deep 40° one. In simulation the deep one reaches 11.7 cm/s and
stays upright 4 times out of 5. On the bench, the shallow crouch was the better of the two (stood
for 27, 35, 46 and 56 s across four runs); the deep one only worked with the trunk held back an
extra 12° from neutral.

## Two candidates with real headroom

Neither is built. Both are geometry changes, not search parameters, so nothing here comes from
the fitting loop — they're where the loop's own numbers point next.

- **Cradle soles** — a sole curved in the frontal plane instead of flat, so the foot *rolls* as
  the hip corrects instead of pivoting on a fixed edge. Turns the 2.7 mm limit above into
  something that scales with the roll angle instead of saturating almost immediately.
- **Two ankle servos** — lets the foot stay level under a rolling trunk instead of tilting with
  it, which is the more direct fix for the same problem. Costs two more servos, two more
  channels, and the packaging and firmware work that come with them.
