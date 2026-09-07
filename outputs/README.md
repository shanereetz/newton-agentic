# TH35 snap-on DIN rail attachment

Printable prototype for a nominal 35 mm top-hat DIN rail. Rail size reference: [OKW TH35, 35 × 7.5 mm](https://www.okw.com/en/In-Box/C7115077.htm). The 1.35 mm flange pocket is a design assumption for an approximately 1 mm flange, not a universal rail specification. Measure your rail before printing.

Files: `din_rail_clip.stl` is the ready-to-slice mesh in millimetres; `din_rail_clip.scad` is editable OpenSCAD source; `din_rail_clip_preview.svg` is a technical preview.

Overall envelope: 39.2 mm across rail × 24 mm along rail × 21 mm deep. Flat mounting back is 39.2 × 24 mm. This is a generic attachment base with no mounting holes; drill or adapt the back for your device. Rail flange seats 17 mm from the back. Fixed hook on the left; flexible arm and sloping catch on the right. Hook engagement is approximately 1.2–1.7 mm per side for a centered nominal rail.

## Printing and use

- Starting print settings: PETG, 0.2 mm layers, 4 perimeters, 100% infill. These settings have not been physically tested.
- Keep the exported orientation: the 39.2 × 21 mm cross-section sits on the bed and the 24 mm rail direction grows vertically. This keeps spring bending in the layer plane. The constant cross-section needs no supports.
- Engage the fixed hook over one flange, then pivot the opposite side down until the ramp passes the other flange and the spring catch returns behind it.
- To remove, push the flexible arm outward, away from the rail center, then lift that side. Do not force the rigid hook.
- Print one sample and check engagement, removal, and spring recovery before attaching equipment. No physical fit, fatigue, temperature, or load validation has been performed. This model has no rated working load.

## Adjustment

Open the SCAD file in OpenSCAD, edit parameters, render, and export STL. `rail_width` adjusts flange spacing; `flange_gap` adjusts flange thickness clearance; `clip_width` sets width along the rail; `arm_thickness` changes spring stiffness. Start with small clearance changes of 0.1 mm. Changing these values requires a fresh fit test.

## Mesh verification

The supplied STL has 42 vertices and 80 triangles. All edges are shared by exactly two triangles with opposite winding; the mesh is a single closed connected extrusion with no overlapping solids. Source defaults reproduce the STL profile. Mechanical behavior remains untested.
