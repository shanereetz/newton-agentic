# DIN-rail and harmonic-drive models

Local design and simulation workspace.

## Demonstrated baseline

`outputs/harmonic_drive_vbd/` contains the Newton 1.5.1 / Warp 1.17.0 VBD contact demo. It was executed on Apple Silicon CPU and produced the saved 7.1-second trajectory, contact metrics, and a reduction near 29:1.

- Open `outputs/harmonic_drive_vbd/replay.html` for the recorded demonstration.
- Follow that folder's README to reproduce the simulation.
- The native viewer entry point is `native_viewer.py`; its physics is the same demo. Native macOS window creation could not be verified from the Codex sandbox.
- Baseline validation includes timestep refinement and disabling circular-spline contact.

This baseline is deliberately simplified and is not a validated engineering gearbox model. Keep it reproducible on `main`. More realistic CUDA experiments belong on a separate branch.

## Other deliverables

- `outputs/harmonic_drive/`: static 3D assembly, printable parts, and model generator.
- `outputs/din_rail_clip.*`: printable DIN-rail attachment and editable OpenSCAD model.

Local environments, caches, scratch files, and redundant ZIP bundles are ignored.
