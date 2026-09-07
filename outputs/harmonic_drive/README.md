# Simple harmonic drive model

A static educational model, 68 mm outside diameter and 25 mm assembled height, containing three separate closed solids:

- Circular spline: rigid ring with 60 simplified internal teeth.
- Flexspline: cup with 58 simplified external teeth and a 10 mm center opening. Its top is permanently modeled as elliptical.
- Wave generator: elliptical insert with an 8 mm input opening. The bearing is represented by the outer surface of the insert; individual rolling elements are omitted.

Open `harmonic_drive.glb` in a glTF-compatible 3D viewer for the color assembly, or `harmonic_drive_exploded.glb` for separated components. STL files use millimetres; GLB files use metres according to the glTF standard. The assembly STL contains three separate shells. Print individual STL parts if desired; translate each part to the bed in your slicer.

`build_model.py` is the editable source. Change the dimensions or radial functions in `parts`, then run it with Python 3 to regenerate the meshes. No external packages are required. The final preview-data export is optional and requires a `work` folder in the current directory.

This is a visual model, not a functional reducer. Tooth shapes are illustrative rather than conjugate profiles, and the fixed elliptical cup cannot reproduce the elastic deformation required by a real harmonic drive. Clearances, tooth interference, bearing construction, materials, torque capacity and print fit have not been engineered or tested.

With a real 60/58 gear pair, a fixed circular spline and wave-generator input, the ideal flexspline output is -1/29 of input speed. Reference: [Harmonic Drive operating principle](https://legacy.harmonicdrive.net/reference/applicationnotes/principles.php).

Validation: all three individual meshes have nonzero-area triangles, positive enclosed volume, and exactly two oppositely oriented triangle edges at every shared edge. This validates closed mesh topology, not mechanical function.
