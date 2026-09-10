# Newton VBD steel-cup experiment — CUDA

This branch contains a runnable candidate for CUDA testing, **not a validated realistic gearbox**. The earlier demonstrated CPU model is preserved on `main` at commit `60032f1`, in `outputs/harmonic_drive_vbd/`. This experiment has not been executed on CUDA or verified in the native viewer on the development Mac.

## Run on your CUDA machine

Copy or clone the local repository, then select `exploration/realistic-cuda`. Use a CUDA-capable NVIDIA machine with a compatible driver and Python (development checks used Python 3.13). From the repository root:

```sh
python -m venv .venv
# Linux:
source .venv/bin/activate
# Windows PowerShell, instead:
# .venv\Scripts\Activate.ps1
python -m pip install -r outputs/harmonic_drive_realistic/requirements-viewer.txt
python -c "import warp as wp; wp.init(); assert wp.is_cuda_available()"
python outputs/harmonic_drive_realistic/realistic.py --viewer
```

For a headless run, omit `--viewer`. The first run compiles kernels and builds mesh SDFs; this may take time. No frame-rate or GPU-memory claim has been measured. Results are saved in `outputs/harmonic_drive_realistic/runs/default/metrics.csv` and `run.json`. Use `--out` for separate runs. The viewer holds the final frame until closed; closing saves metrics. `--record-mesh` additionally saves a potentially large trajectory.

## What changed

- A three-dimensional steel flexspline cup, bottom diaphragm, and freely rotating supported hub replace the soft demonstration ring. Reference properties: E = 206 GPa, Poisson ratio 0.3, density 7,850 kg/m³.
- An elastic bearing outer race sits between the cup and the prescribed elliptical rolling-support envelope. Ideal axial retention is enforced; individual rolling elements and cage dynamics are omitted.
- Assembly is staged: cam insertion during 0–2 s, circular-spline insertion during 2.5–4.5 s, output torque ramp to 0.1 N·m during 4.5–5 s, then input rotation ramps toward 0.1 rad/s. Output rotation is determined by contact and deformation.
- Newton 1.5.1 `SolverVBD` supplies tetrahedral elasticity and self-contact. Native CUDA mesh SDFs and full-surface rigid/soft contacts include edge and face candidates. A pinned solver subclass supplies ideal supports and tributary-area weighting of rigid/soft penalty forces.
- Metrics include element Jacobians, strain, elastic energy, von Mises stress, contact penetration, contact counts, and normal reaction torque. Inverted elements terminate the run.

`design.json` uses SI units. Geometry has 58/60 teeth, nominal 29:1 reduction, and a 68 mm circular-spline outside diameter. These are reference design choices, not measured manufacturer data.

## Accuracy limits and checks

Generic involute teeth with sharp profile joins are used; their conjugacy under cup deformation has not been established. They are not a manufacturer's harmonic-drive tooth form. Linear tetrahedra need mesh refinement, particularly through the wall, at roots, and at the diaphragm transition. Stress peaks at sharp corners are not reliable design stresses. The material is elastic, with no plasticity or fatigue model.

Rigid/soft contact area weighting is a custom approximation. Cup/race self-contact uses Newton's nodal penalty approximation, with a 2 μm contact radius and stiffness scaled by representative nodal area. Friction coefficients are assumptions. Pressure metrics are penalty estimates, not independently validated Hertz pressures. Reported reaction torques include normal rigid/soft forces only, not friction or a complete torque balance.

The default 25 μm SDF voxel size must be refined against 40 μm nominal backlash. The default timestep and 100 VBD iterations are starting points, not established convergence settings. Physical steel stiffness makes convergence substantially harder than the old soft demo. Stability alone does not demonstrate accuracy.

Run independent refinements on CUDA:

```sh
python outputs/harmonic_drive_realistic/convergence.py --dry-run
python outputs/harmonic_drive_realistic/convergence.py
```

The sweep changes timestep, iteration count, SDF resolution, and mesh density separately, plus removes circular-spline contact as a control. It saves logs and `comparison.json`. Inspect complete time histories, loaded transmission, penetration, and energy as well as reported differences. The short run includes transients; it does not establish steady-state performance or fatigue. Contact-penalty, friction, assembly-speed, load, and longer-duration sweeps remain necessary. An independent implicit finite-element contact benchmark and experimental calibration are still outstanding. Do not tune toward 29:1 alone and call that validation.

Local checks (CPU only):

```sh
python outputs/harmonic_drive_realistic/realistic.py --geometry-only
python outputs/harmonic_drive_realistic/check_kernels.py
```

These passed on the development Mac: watertight topology, positive reference tetrahedral volumes, tooth counts, torque distribution, Warp CPU compilation of custom kernels, support constraints, assembly staging, and nonduplicated tributary area. They do not test CUDA SDF construction, full solver integration, mechanical convergence, or native window creation.

## References

- [Newton VBD API](https://newton-physics.github.io/newton/1.5.0/api/_generated/newton.solvers.SolverVBD.html) and [collision pipeline](https://newton-physics.github.io/newton/1.5.0/api/_generated/newton.CollisionPipeline.html); implementation is pinned to installed 1.5.1 source because subclass hooks are internal.
- [Flexible strain-wave transmission modeling](https://link.springer.com/article/10.1186/s10033-023-00909-2): cup and bearing flexibility and surface contact motivate this upgrade.
- [Steel flexspline material reference](https://www.mdpi.com/2076-0825/15/7/402); material values here still require calibration to the intended part.
- [Abaqus contact constraint methods](https://docs.software.vt.edu/abaqusv2025/English/SIMACAEITNRefMap/simaitn-c-contactconstraints.htm) for an independent finite-element comparison. No Abaqus benchmark has been run.

## Reading input and output in the native viewer

The **wave generator is the input**, the **flexspline is the output**, and the outer circular spline stays fixed. The deforming oval travels much faster than the flexspline material rotates.

- **Amber pointer:** wave-generator input angle.
- **Cyan pointer and three-spoke dial:** measured flexspline output angle, representing an output shaft. This is a display indicator, not an additional simulated part.
- **White index and surrounding scale:** fixed outer-ring reference, with ticks every 10 degrees.
- **Pink dot:** follows the same flexspline material vertex, making slow material rotation distinguishable from the traveling deformation.

The sidebar identifies each role and shows angles in degrees. Output angles are unwrapped across full turns. All indicators show actual motion without amplification or an imposed gear ratio. Add `--top-view` to the native-viewer command to look along the shaft. Physics and the drive arrangement are unchanged.
