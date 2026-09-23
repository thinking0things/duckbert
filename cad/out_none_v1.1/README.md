# Printable files, v1.1

The CAD source, exported STL and pre-arranged print plates for the current headless (no-neck)
Duckbert body. This is the same v1.1 revision the simulation imports (see the main
[README](../../README.md#model-and-controller) and `dist/assets/provenance.json`).

- `Microduck_SG90_headless_v1_1.FCStd` — the FreeCAD source.
- `stl/` — one STL per part, plus `00_fit_coupon.stl` (a small print to check your printer's
  tolerances before committing a full plate).
- `plates/` — the parts pre-arranged onto three build plates by colour (`charcoal`, `cream`,
  `orange`), with `layout.json` describing which part is on which plate.

**Not the final revision.** Printed as-is, this body has three known issues that the next
revision fixes:

1. The lid does not sit flush with the trunk tub.
2. The battery bay is an early layout, not yet optimised for the LiPo cassette.
3. Assembly currently needs glue to close; the next revision closes without it.

If you print this version, expect to work around those three. Bill of materials, electronics
and firmware are documented in the project this repository accompanies.
