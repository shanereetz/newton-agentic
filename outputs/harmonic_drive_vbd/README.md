# Harmonic drive contact simulation — Newton VBD

This simulation actually runs **Newton Physics 1.5.1, `newton.solvers.SolverVBD`**, with Warp 1.17.0. It was executed and tested on an Apple Silicon Mac using the CPU backend. The browser file is a replay of recorded Newton particle positions, not a browser physics substitute.

Open `replay.html` to play the full recorded run, scrub time, switch between top and 3D views, and highlight contact points. Drag the 3D view to rotate it. The dark dot marks a material point on the flexspline. Playback starts paused and does not loop. No network connection is needed for replay.

## Model

- Fixed circular spline: 60 teeth, 68 mm outside diameter.
- Flexspline: 58 teeth, 928 free particles, 1,392 tetrahedral finite elements. VBD computes deformation and rotation. There is **no prescribed output angle, gear-ratio constraint, or per-frame ellipse assignment**.
- Wave generator: kinematically driven elliptical rigid mesh. This represents an ideal speed-controlled motor and the outside envelope of its bearing. It presses against the flexspline through Newton particle-to-mesh contacts.
- The ring is initially placed in an approximate ellipse once as an assembly preload, then settles for 0.5 s. Newton alone updates it thereafter.
- A 4 mm thick toothed ring replaces the prior display model's cup to keep the contact experiment small. Gravity is disabled. There is no output shaft, applied output load, axial bearing, or motor torque limit.
- SI units in the solver. Young's modulus 2 MPa, Poisson ratio 0.3, density 1,200 kg/m³, contact stiffness 100,000 N/m, friction coefficient 0.05. This deliberately compliant demonstration material does not represent a commercial steel flexspline.
- Particle collision radius 0.025 mm; contact search margin 0.15 mm. Contact is sampled at vertices; full-surface SDF and self-contact are disabled. Coarse sinusoidal tooth profiles are illustrative, not manufactured conjugate profiles.

The highlighted points are those with positive penetration of the particle contact envelope at a recorded frame. `max_contact_penalty_N` estimates the normal elastic penalty as stiffness × penetration; it is not a calibrated pressure or a complete force including friction/damping. Metrics are sampled at recorded frames, not every solver substep.

## Results and checks

The 7.1 s run uses 8 substeps per 30 Hz frame (240 solver steps/s), 15 VBD iterations per step, and a smooth ramp to 1 rad/s input. It took about 52 s of CPU execution with cached kernels.

- Regression of settled output against input: **−0.0344722 rad/rad**, or **29.009:1** reduction; ideal 60/58 gearing is −1/29.
- Both cam and tooth contact were detected. No inverted tetrahedra at recorded frames; the minimum sampled volume ratio was 0.985 of rest volume.
- Maximum sampled contact-envelope penetration after settling: **0.000385 mm**. Initial preload overlap reached 0.0665 mm. These are vertex-based metrics, not a whole-surface penetration guarantee.
- A 3 s refinement run with 16 substeps/frame and 20 iterations differed from the coarse output by **0.0155°**.
- Disabling circular-spline collision changed 3 s output from **−4.33° to +13.51°**. This verifies that outer tooth contact materially determines the output rather than an imposed ratio.

Raw results are in `results/metrics.csv` and `results/summary.json`. The trajectory is in `results/trajectory.json`. Comparisons and assertions are in `validation/`. These checks support a contact-driven demonstration; they do not establish engineering accuracy, load capacity, fatigue, or torque transmission performance.

## Run again

Tested with Python 3.13. From this folder:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python simulate.py --duration 7.1
.venv/bin/python build_viewer.py
```

On Windows use `.venv\Scripts\python.exe` for the Python commands. On a supported NVIDIA/CUDA machine, add `--device cuda:0`; CUDA execution has not been tested for this project. The default CPU configuration is the validated path.

Useful controls:

```sh
# Finer integration
.venv/bin/python simulate.py --duration 3 --substeps 16 --iterations 20 --out refined
# Remove outer-ring contact to inspect causality
.venv/bin/python simulate.py --duration 3 --no-ring-contact --out no_ring
# Change input speed, material stiffness, or angular mesh resolution
.venv/bin/python simulate.py --speed 0.5 --young 2000000 --samples 8
```

Other options: `--friction`, `--contact-ke`, `--settle`, `--fps`, `--iterations`, `--device`, `--out`. Altered configurations need fresh validation. Python dependencies install only into your chosen virtual environment. Warp writes compilation artifacts under `.warp_cache` beside the script; override with `WARP_CACHE_PATH` if needed. A small-element volume warning is expected from Newton's absolute mesh-quality threshold at this millimetre scale; signed volumes are also checked during the run.

Sources: [Newton VBD API](https://newton-physics.github.io/newton/1.5.0/api/_generated/newton.solvers.SolverVBD.html), [Newton rigid/soft example](https://github.com/newton-physics/newton/blob/v1.5.1/newton/examples/multiphysics/example_rigid_soft_contact.py), [Harmonic Drive operating principle](https://legacy.harmonicdrive.net/reference/applicationnotes/principles.php).

## Native Newton visualizer (live)

On the prepared Mac, double-click `Open Newton Viewer.command` in Finder. It uses the already-installed workspace environment and opens Newton's `ViewerGL`, integrating the VBD model live. Pause/resume with the viewer's play control or Space. Close the window to stop. CPU execution is slower than real time.

The Codex sandbox could not enumerate a macOS display, so native-window startup must be performed from the desktop session. The physics simulation was validated; native rendering could not be verified inside that sandbox.

For a fresh installation, install `requirements-viewer.txt` in your Python environment, then run `python native_viewer.py`. The provided `.command` launcher points to this Mac's prepared environment.
