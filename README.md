# DIN-rail and harmonic-drive models

Local design and simulation workspace.

## Harmonic drive on this branch

`outputs/harmonic_drive_vbd/simulate.py` and its native viewer now use ROM elastics: nonlinear tetrahedral elasticity projected into a Fourier displacement basis. The default uses 102 reduced coordinates for the existing 928-vertex ring. Newton provides collisions and visualization; the reduced solve runs on CPU, including when collisions use CUDA.

Follow [the ROM README](outputs/harmonic_drive_vbd/README.md) for setup, execution, and validation. New recordings use `results_rom/` and `replay_rom.html`. The historical directory name and saved VBD results are retained; the VBD implementation remains reproducible on `main`. The steel-cup experiment in `outputs/harmonic_drive_realistic/` is separate and still uses VBD.

This is an experimental, compliant contact model, not a validated engineering gearbox.

## Other deliverables

- `outputs/harmonic_drive/`: static 3D assembly, printable parts, and model generator.
- `outputs/din_rail_clip.*`: printable DIN-rail attachment and editable OpenSCAD model.

Local environments, caches, scratch files, and redundant ZIP bundles are ignored.
