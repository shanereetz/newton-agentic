# Harmonic drive — ROM elastics

On this branch, `simulate.py` and `native_viewer.py` use the local `SolverROM` in `rom_elastics.py`. Newton 1.5.1 and Warp 1.17.0 supply the mesh, collision candidates, state buffers and renderer. SciPy integrates reduced-order elasticity on the CPU. `--device cuda:0` moves Newton collision detection and state buffers to CUDA; it does **not** move the reduced solve to the GPU.

The directory keeps its historical name to preserve existing asset paths. `replay.html`, `results/`, `results_gpu/`, `refined/` and `validation/` contain earlier VBD evidence; their numerical claims do not validate ROM. See [README_VBD.md](README_VBD.md) for that history and use `main` to reproduce VBD. The separate `../harmonic_drive_realistic/` steel-cup VBD experiment is not the ROM implementation.

## Run

From this folder:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
# Small dense ROM operations benefit from one BLAS thread.
export OPENBLAS_NUM_THREADS=1
.venv/bin/python simulate.py --duration 3
.venv/bin/python build_viewer.py
```

Open `replay_rom.html` for the self-contained recorded replay. New runs default to `results_rom/` and write `trajectory.json`, `metrics.csv`, and `summary.json`, with ROM solver identification and configuration. Use `--out` for separate runs and `build_viewer.py --data path/trajectory.json --out path/replay.html` to select another recording. Replay labels are taken from the actual recording, including when replaying historical VBD data.

For Newton's live viewer on a graphical desktop:

```sh
.venv/bin/python -m pip install -r requirements-viewer.txt
.venv/bin/python native_viewer.py --device cuda:0 --top-view
```

The viewer accepts the same material, mesh, contact and ROM options as the headless simulation. Space pauses; close the window to stop. The amber pointer is the wave-generator input, the cyan dial is measured flexspline output, and the pink dot follows a material vertex. These indicators do not impose a gear ratio.

## Elasticity and contact

The 60/58-tooth demonstration geometry is retained: a fixed circular spline, kinematically driven elliptical cam, and deformable 4 mm thick ring with 928 vertices and 1,392 tetrahedra. Young's modulus is 2 MPa, Poisson ratio 0.3, and density 1,200 kg/m³. There is no output shaft, applied load, gravity, self-contact, or full-surface contact. The cam diameters are 50.7 × 45.6 mm; `--cam-minor 0.02362` selects the historical minor semiaxis.

The displacement is `x = X + B q`. A geometric Fourier basis with independent radial variation and axial thickness modes reduces 2,784 vertex coordinates to 102 coordinates at `--rom-harmonics 8`. In-plane affine deformation, finite rotation about the shaft and the initial assembly ellipse are representable. The basis is built from rest geometry, without a training trajectory or prescribed output angle.

All tetrahedra contribute stable Neo-Hookean energy and analytic reduced forces. The material parameters match Newton's small-strain Lamé conversion. Implicit Euler minimizes inertia, elastic energy, unilateral normal penalties and regularized friction using a preconditioned BFGS solve. Friction uses lagged normal loads. Damping is mass-proportional (`--rom-damping`, in 1/s); the VBD element/contact damping is not used by ROM. Newton contact candidates and tangent planes are frozen within each substep and refreshed for the next one.

`--iterations` (default 400) limits each nonlinear solve, and `--rom-tolerance` controls its preconditioned gradient tolerance. A failed convergence check, nonfinite state, contact-buffer overflow, or inverted element stops execution. The CSV records reduced dimension, final-substep iterations and residual. Frame contact penalties are stiffness × vertex penetration, not calibrated pressure; recorded extrema do not bound between-frame penetration.

This is a local Galerkin ROM, not an upstream Newton ROM API. Elastic quadrature still visits every tet and collision detection visits the full mesh, so fewer coordinates alone do not establish a speedup. The truncated basis omits local tooth and out-of-plane bending modes. Neither the old VBD reduction ratio nor commercial steel-gearbox accuracy is assumed for this solver.

## Validation

```sh
OPENBLAS_NUM_THREADS=1 .venv/bin/python -m unittest discover -s . -p 'test_rom*.py'
# Contact causality, temporal refinement, and basis refinement:
OPENBLAS_NUM_THREADS=1 .venv/bin/python simulate.py --duration 3 --no-ring-contact --out no_ring_rom
OPENBLAS_NUM_THREADS=1 .venv/bin/python simulate.py --duration 3 --substeps 16 --out refined_rom
OPENBLAS_NUM_THREADS=1 .venv/bin/python simulate.py --duration 3 --rom-harmonics 12 --out modes12_rom
```

The physics tests check energy derivatives, finite-rotation invariance, preload reconstruction, stationary unforced rest, kinematic cam propagation, and an integrated contact response. Compare basis size and timestep before using the output quantitatively. The simulation remains a compliant contact demonstration rather than a validated gearbox design.

Measured checks on Linux with the pinned dependencies are recorded in [validation_rom.json](validation_rom.json):

- The 3 s default run measured output/input = −0.0343073, versus ideal −1/29.
- Disabling circular-spline contact changed the fitted slope to +0.998081.
- Doubling substeps from 8 to 16 changed final output by 0.00090°.
- Increasing Fourier harmonics from 8 to 12 (102 to 150 coordinates) changed final output by 0.00297°.
- Six physics tests and a 0.2 s CUDA-collision assembly smoke test passed. The native viewer import and command-line path were checked; its graphical window was not exercised.

The raw ROM recordings are generated locally and ignored by Git. Run configurations, summary metrics and trajectory hashes are retained in the validation report. Wall timings include initialization and concurrent validation workloads and should not be interpreted as a performance comparison with VBD.
