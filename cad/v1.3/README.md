# Duckbert v1.3 CAD

The design sources for the parts in [`../../print`](../../print).

- `Duckbert_v1_3.FCStd`: the FreeCAD assembly (r3 body tub, the r2 hip block, legs, boards as envelopes).
- `step/`: one STEP per part, the same parts as the STLs. `step/shoes/` holds the R80 shoe kit.
- `hip_block_r2/`: a stiffer hip block. It bolts to the body and battery cassette with the same holes as
  the printed one, and each servo housing is supported by a continuous wall instead of its front
  face alone. Not printed yet.

The older `../out_none_v1.1` is the earlier body the browser simulation was built from.
