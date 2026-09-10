# Harmonic drive contact simulation — Newton VBD

This project runs **Newton Physics 1.5.1**, `newton.solvers.SolverVBD`, and Warp 1.17.0. Pass `--device cuda:0` to run collision detection and VBD integration on the first NVIDIA GPU. The CUDA path has been smoke-tested on an NVIDIA RTX PRO 6000 Blackwell GPU; the checked-in numerical results were produced and validated separately on the CPU backend.

The browser viewer is a replay of recorded Newton particle positions, not a browser physics substitute. Open `replay.html` to play the run, scrub time, switch between top and 3D views, and highlight contact points. Drag the 3D view to rotate it. The dark dot marks a material point on the flexspline. Playback starts paused and does not loop. No network connection is needed for replay.

## Run on an NVIDIA GPU

The host needs a working NVIDIA driver and a CUDA-capable GPU. Warp ships the CUDA runtime pieces used by this project, so a separate CUDA Toolkit installation is not normally required.

From this folder, create the environment and confirm that Warp can see `cuda:0`:

```sh
nvidia-smi
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -c "import warp as wp; wp.init(); print(wp.get_device('cuda:0'))"
```

Run the full recorded simulation on the GPU and rebuild the offline replay:

```sh
.venv/bin/python simulate.py --device cuda:0 --duration 7.1 --out results_gpu
.venv/bin/python build_viewer.py --data results_gpu/trajectory.json
```

Then open `replay.html` in a browser. The first CUDA run may spend additional time compiling kernels; later runs reuse the cache under `.warp_cache/`.

On Windows, use `.venv\Scripts\python.exe` in place of `.venv/bin/python`. If the machine has multiple NVIDIA GPUs, select one with `--device cuda:1`, `cuda:2`, and so on. Use `--device cpu` only when a CUDA device is unavailable or when intentionally comparing backends.

## Model

- Fixed circular spline: 60 teeth, 68 mm outside diameter.
- Flexspline: 58 teeth, 928 free particles, 1,392 tetrahedral finite elements. VBD computes deformation and rotation. There is **no prescribed output angle, gear-ratio constraint, or per-frame ellipse assignment**.
- Wave generator: kinematically driven elliptical rigid mesh. This represents an ideal speed-controlled motor and the outside envelope of its bearing. It presses against the flexspline through Newton particle-to-mesh contacts.
- The ring is initially placed in an approximate ellipse once as an assembly preload, then settles for 0.5 s. Newton alone updates it thereafter.
- A 4 mm thick toothed ring replaces the prior display model's cup to keep the contact experiment small. Gravity is disabled. There is no output shaft, applied output load, axial bearing, or motor torque limit.
- SI units in the solver. Young's modulus 2 MPa, Poisson ratio 0.3, density 1,200 kg/m³, contact stiffness 100,000 N/m, friction coefficient 0.05. This deliberately compliant demonstration material does not represent a commercial steel flexspline.
- Particle collision radius 0.025 mm; contact search margin 0.15 mm. Contact is sampled at vertices; full-surface SDF and self-contact are disabled. Coarse sinusoidal tooth profiles are illustrative, not manufactured conjugate profiles.

The highlighted points are those with positive penetration of the particle contact envelope at a recorded frame. `max_contact_penalty_N` estimates the normal elastic penalty as stiffness × penetration; it is not a calibrated pressure or a complete force including friction/damping. Metrics are sampled at recorded frames, not every solver substep.

## Checked-in results and checks

The checked-in 7.1 s run uses 8 substeps per 30 Hz frame (240 solver steps/s), 15 VBD iterations per step, and a smooth ramp to 1 rad/s input. It took about 52 s of CPU execution with cached kernels. Treat a newly generated CUDA result as a new run and repeat the checks before making quantitative claims from it.

- Regression of settled output against input: **−0.0344722 rad/rad**, or **29.009:1** reduction; ideal 60/58 gearing is −1/29.
- Both cam and tooth contact were detected. No inverted tetrahedra at recorded frames; the minimum sampled volume ratio was 0.985 of rest volume.
- Maximum sampled contact-envelope penetration after settling: **0.000385 mm**. Initial preload overlap reached 0.0665 mm. These are vertex-based metrics, not a whole-surface penetration guarantee.
- A 3 s refinement run with 16 substeps/frame and 20 iterations differed from the coarse output by **0.0155°**.
- Disabling circular-spline collision changed 3 s output from **−4.33° to +13.51°**. This verifies that outer tooth contact materially determines the output rather than an imposed ratio.

Raw results are in `results/metrics.csv` and `results/summary.json`. The trajectory is in `results/trajectory.json`. Comparisons and assertions are in `validation/`. These checks support a contact-driven demonstration; they do not establish engineering accuracy, load capacity, fatigue, or torque transmission performance.

## Additional GPU runs

```sh
# Finer integration on the first NVIDIA GPU
.venv/bin/python simulate.py --device cuda:0 --duration 3 --substeps 16 --iterations 20 --out refined_gpu

# Remove outer-ring contact to inspect causality
.venv/bin/python simulate.py --device cuda:0 --duration 3 --no-ring-contact --out no_ring_gpu

# Change input speed, material stiffness, or angular mesh resolution
.venv/bin/python simulate.py --device cuda:0 --speed 0.5 --young 2000000 --samples 8 --out parameter_sweep_gpu
```

Each `--out` directory receives its own `trajectory.json`, `metrics.csv`, and `summary.json`. To replay a non-default trajectory, pass its path to `build_viewer.py`; the builder still writes `replay.html` in the project directory.

Other options include `--friction`, `--contact-ke`, `--settle`, `--fps`, `--iterations`, `--device`, and `--out`. Altered configurations and backend changes need fresh validation. Python dependencies install only into the virtual environment. Set `WARP_CACHE_PATH` to relocate Warp's compilation cache if needed. A small-element volume warning is expected from Newton's absolute mesh-quality threshold at this millimetre scale; signed volumes are also checked during the run.

Sources: [Newton VBD API](https://newton-physics.github.io/newton/1.5.0/api/_generated/newton.solvers.SolverVBD.html), [Newton rigid/soft example](https://github.com/newton-physics/newton/blob/v1.5.1/newton/examples/multiphysics/example_rigid_soft_contact.py), [Harmonic Drive operating principle](https://legacy.harmonicdrive.net/reference/applicationnotes/principles.php).

## Native Newton visualizer on the GPU

The live viewer needs a graphical desktop and the additional viewer dependencies:

```sh
.venv/bin/python -m pip install -r requirements-viewer.txt
.venv/bin/python native_viewer.py --device cuda:0
```

This keeps rendering in Newton's `ViewerGL` while the simulation state is integrated on `cuda:0`. Pause or resume with the viewer's play control or Space, and close the window to stop. The included `Open Newton Viewer.command` is a prepared, machine-specific macOS launcher; use the explicit command above for a portable GPU-backed launch.

## Reading input and output in the native viewer

The **wave generator is the input**, the **flexspline is the output**, and the outer circular spline stays fixed. The deforming oval travels much faster than the flexspline material rotates.

- **Amber pointer:** wave-generator input angle.
- **Cyan pointer and three-spoke dial:** measured flexspline output angle, representing an output shaft. This is a display indicator, not an additional simulated part.
- **White index and surrounding scale:** fixed outer-ring reference, with ticks every 10 degrees.
- **Pink dot:** follows the same flexspline material vertex, making slow material rotation distinguishable from the traveling deformation.

The sidebar identifies each role and shows angles in degrees. Output angles are unwrapped across full turns. All indicators show actual motion without amplification or an imposed gear ratio. Add `--top-view` to the native-viewer command to look along the shaft. Physics and the drive arrangement are unchanged.

### More visible rigid wave generator

The live CPU demo now uses a rigid elliptical cam with diameters 50.7 × 45.6 mm (previously 50.7 × 47.24 mm). Its amber outline and axis marks rotate rigidly with the input. Only the toothed flexspline is deformable in this simplified scene. The cyan output dial is a separate display overlay above the assembly, not the cam surface.

Use `--cam-minor 0.02362` with `native_viewer.py` or `simulate.py` to restore the earlier cam geometry. Changing the cam changes contact conditions; saved replay files still show their original geometry and results. This visualization experiment is separate from the steel-cup CUDA model, which additionally models bearing-race flexibility.
