# Printable files, v1.1

The CAD source, exported STL and a print plate for the current headless (no-neck) Duckbert
body. This is the exact v1.1 revision the simulation imports — every mesh here hashes to the
one recorded in `dist/assets/provenance.json` — so it's what you see walking on the site, see
the main [README](../../README.md#model-and-controller).

- `Microduck_SG90_headless_v1_1.FCStd` — the FreeCAD source.
- `stl/` — one STL per part, plus `00_fit_coupon.stl` (a small print to check your printer's
  tolerances before committing a full plate).
- `plates/plate_all_parts.stl` — every part pre-arranged on one build plate, ready to slice.

The battery bay in this revision is a drawer between the hip block and the trunk (four M2 down
into the block, four M2 up into the trunk floor) rather than the earlier cassette bolted above
the hip block — a step toward fixing that compromise, not the final layout.

**Not the final revision.** Printed as-is, this body has three known issues that the next
revision fixes:

1. The lid does not sit flush with the trunk tub.
2. The battery bay is still being reworked (see above).
3. Assembly currently needs glue to close; the next revision closes without it.

If you print this version, expect to work around those three. Bill of materials, electronics
and firmware are documented in the project this repository accompanies.
