"""Display-only measured-motion indicators shared by the Newton viewers."""
import math
import numpy as np
import warp as wp


class MotionOverlay:
    def __init__(self, rest, device):
        # Track one material vertex on the tooth rim, not the moving wave crest.
        candidates = np.flatnonzero(rest[:, 2] > rest[:, 2].max() - 1e-6)
        self.marker = int(min(candidates, key=lambda i: abs(math.atan2(rest[i, 1], rest[i, 0]))))
        self.device = device
        self.previous = None
        self.output = 0.
        self.input = 0.

    def panel(self, imgui):
        imgui.separator()
        imgui.text("HARMONIC DRIVE: INPUT / OUTPUT")
        imgui.text(f"AMBER: wave generator INPUT {math.degrees(self.input):.1f} deg")
        imgui.text(f"CYAN: flexspline OUTPUT {math.degrees(self.output):.2f} deg")
        imgui.text("WHITE: outer circular spline FIXED (0 deg)")
        imgui.text("PINK: one material point on the flexspline")
        imgui.text("Cyan dial represents the output shaft angle.")
        imgui.text("Pointers show actual angles, with no magnification.")
        imgui.text("The moving oval wave is not the output rotation.")

    def draw(self, viewer, q, input_angle, output_angle):
        if self.previous is None:
            self.output = output_angle
        else:
            self.output += math.atan2(math.sin(output_angle-self.previous), math.cos(output_angle-self.previous))
        self.previous = output_angle
        self.input = input_angle
        z = .009
        starts, ends, colors = [], [], []
        def line(a, b, color):
            starts.append(a); ends.append(b); colors.append(color)
        def point(r, angle):
            return [r*math.cos(angle), r*math.sin(angle), z]
        # Stationary scale outside the circular spline; every tick is 10 degrees.
        for i in range(36):
            a = i*math.tau/36
            line(point(.0355, a), point(.038 if i%9==0 else .0365, a), (.55,.55,.6))
        line(point(.030, 0), point(.041, 0), (1.,1.,1.))
        # Actual angles, never multiplied or replaced with the theoretical ratio.
        for angle, radius, color in [(input_angle,.032,(1.,.6,.08)), (self.output,.040,(.1,1.,1.))]:
            line([0,0,z], point(radius,angle), color)
            tip = np.array(point(radius,angle))
            back = np.array(point(radius-.003,angle))
            side = .0015*np.array([-math.sin(angle), math.cos(angle),0])
            line(tip,back+side,color);line(tip,back-side,color)
        # Display-only output shaft dial. Driven by measured flexspline rotation;
        # it adds no rigid body, contact, inertia, or kinematic constraint.
        for a,b in zip(np.linspace(0,math.tau,65)[:-1],np.linspace(0,math.tau,65)[1:]):
            line(point(.010,a),point(.010,b),(.1,1.,1.))
        for offset in (0,math.tau/3,2*math.tau/3):
            line(point(.002,self.output+offset),point(.010,self.output+offset),(.1,1.,1.))
        # Arc displays the fractional output turn relative to fixed zero.
        arc = np.linspace(0, math.fmod(self.output, math.tau), 65)
        for a,b in zip(arc[:-1],arc[1:]):
            line(point(.039,a),point(.039,b),(.1,1.,1.))
        marker = np.array(q[self.marker], copy=True)
        marker[2] += .0008
        line(marker, [marker[0],marker[1],z], (1.,.2,.7))
        arr = lambda data: wp.array(np.asarray(data,dtype=np.float32),dtype=wp.vec3,device=self.device)
        viewer.log_lines('Motion: white fixed / amber input / cyan output',arr(starts),arr(ends),arr(colors))
        viewer.log_points('Pink: same flexspline material point',arr([marker]),radii=.0008,colors=arr([(1.,.2,.7)]))
        viewer.log_scalar('Fixed circular spline (deg)',0.)
        viewer.log_scalar('Flexspline output vs fixed ring (deg)',math.degrees(self.output))
        viewer.log_scalar('Input vs flexspline output (deg)',math.degrees(input_angle-self.output))
        viewer.renderer.set_title(f'Newton | Fixed ring: white 0 deg | Input: amber {math.degrees(input_angle):.1f} deg | Flexspline OUTPUT: cyan {math.degrees(self.output):.2f} deg | Material point: pink')
