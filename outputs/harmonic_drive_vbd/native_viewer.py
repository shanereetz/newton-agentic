"""Live Newton ViewerGL + SolverVBD. Space pauses; close the window to stop."""
import argparse
import math
import time
from pathlib import Path
import warp as wp
from simulate import Simulation
from newton.viewer import ViewerGL


def _patch_pyglet_x11_primary_screen():
    try:
        from pyglet.display import xlib
    except ImportError:
        return
    if not hasattr(xlib.XlibScreen, 'is_primary'):
        xlib.XlibScreen.is_primary=property(lambda self: False)


def main():
    _patch_pyglet_x11_primary_screen()
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--speed',type=float,default=1.)
    parser.add_argument('--paused',action='store_true')
    args=parser.parse_args()
    args.samples=4;args.young=2e6;args.contact_ke=1e5;args.friction=.05
    args.no_ring_contact=False;args.iterations=15;args.fps=30;args.substeps=8;args.settle=.5
    sim=Simulation(args)
    viewer=ViewerGL(width=1280,height=850,vsync=True,paused=args.paused)
    viewer.set_model(sim.model)
    viewer.renderer.set_title('Newton VBD — Live Harmonic Drive')
    viewer.show_ground=False
    viewer.show_particles=False
    # The gearbox is only 68 mm across. ViewerGL's default orbit pivot is
    # metres away, so explicitly target the assembly and use millimetre-scale
    # navigation values.
    viewer.camera.near=.0005
    viewer.camera.far=2.
    viewer.camera_speed=.008
    viewer.set_camera(pos=wp.vec3(.025,-.07,.115),pitch=-58.,yaw=90.)
    viewer.camera.look_at((0.,0.,0.))
    print(f'NATIVE_VIEWER_READY: Newton ViewerGL; live SolverVBD; {args.device}',flush=True)
    frame=0
    try:
        while viewer.is_running():
            start=time.perf_counter()
            if viewer.should_step():
                sim.step()
            viewer.begin_frame(sim.t)
            viewer.log_state(sim.a)
            m=sim.rows[-1]
            viewer.log_scalar('Input (degrees)',math.degrees(m['input_rad']))
            viewer.log_scalar('Output (degrees)',math.degrees(m['output_rad']))
            viewer.log_scalar('Tooth contacts',m['ring_contacts'])
            viewer.log_scalar('Cam contacts',m['cam_contacts'])
            viewer.end_frame()
            frame+=1
            if frame%30==0:
                print(f"Live VBD: t={sim.t:.2f}s, ring contacts={m['ring_contacts']}, output={math.degrees(m['output_rad']):.2f} deg",flush=True)
            # This is live execution, so retain only current diagnostics.
            sim.frames[:]=sim.frames[-1:];sim.rows[:]=sim.rows[-1:]
            time.sleep(max(0.,1/60-(time.perf_counter()-start)))
    finally:
        viewer.close()

if __name__=='__main__':main()
