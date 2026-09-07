// TH35 snap-on attachment prototype. All dimensions in mm.
// Print on the extrusion end (as exported). Flat mounting face: y=0.
rail_width = 35;
side_clearance = 0.30; // total nominal extra width
flange_gap = 1.35; // tune to measured flange thickness + clearance
clip_width = 24; // along rail
arm_thickness = 1.4;
rail_seat = 17;
base_thickness = 3;
h = rail_width/2 + side_clearance/2 + 0.15;
r = h + arm_thickness;
s = rail_seat;
g = flange_gap;
linear_extrude(height=clip_width, convexity=10)
polygon(points=[[-h-2.2,0],[r,0],[r,s+3.7],[h,s+3.7],
 [h-1.5,s+1.8],[h-1.5,s+g],[h,s+g],[h,base_thickness+1],
 [h-1,base_thickness],[h-2.8,base_thickness],[h-2.8,s],
 [h-4.1,s],[h-4.1,base_thickness],[-h+4.1,base_thickness],
 [-h+4.1,s],[-h+0.1,s],[-h+0.1,s+g],[-h+2,s+g],
 [-h+2,s+3.3],[-h+0.1,s+4],[-h-2.2,s+4]]);
