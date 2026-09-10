"""Newton VBD harmonic-drive contact experiment (SI units).
A deformable toothed ring replaces the static display mesh. No output gearing constraint.
"""
from pathlib import Path
import argparse, json, math, time, csv, os
import numpy as np
import warp as wp
import newton
from newton.solvers import SolverVBD

ROOT = Path(__file__).resolve().parent
wp.config.kernel_cache_dir = os.environ.get('WARP_CACHE_PATH', str(ROOT / '.warp_cache'))
newton.use_coord_layout_targets = True
TAU = 2 * math.pi

def ellipse(a,b,t):
    return a*b/np.sqrt((b*np.cos(t))**2+(a*np.sin(t))**2)

def ring_mesh(n, sections):
    t=np.arange(n)*TAU/n
    verts=np.concatenate([np.stack([r(t)*np.cos(t),r(t)*np.sin(t),np.full(n,z)],axis=1) for z,r in sections])
    faces=[]
    for k in range(len(sections)):
        l=(k+1)%len(sections)
        for i in range(n):
            j=(i+1)%n;a=k*n+i;b=k*n+j;c=l*n+j;d=l*n+i
            faces.extend([[a,b,c],[a,c,d]])
    return verts.astype(np.float32), np.asarray(faces,dtype=np.int32)

def flex_mesh(samples):
    n=58*samples;t=np.arange(n)*TAU/n
    outer=.0258+.0009*(1+np.cos(58*t))/2
    vertices=[]
    for z in [-.002,.002]:
        for r in [np.full(n,.0245),outer]:
            vertices.extend(np.stack([r*np.cos(t),r*np.sin(t),np.full(n,z)],axis=1))
    vertices=np.asarray(vertices,dtype=np.float32)
    # Consistent six-tet hexahedron split, periodic at angular seam.
    tets=[]
    pattern=[(0,1,3,7),(0,3,2,7),(0,2,6,7),(0,6,4,7),(0,4,5,7),(0,5,1,7)]
    for i in range(n):
        j=(i+1)%n
        cell=[i,j,n+i,n+j,2*n+i,2*n+j,3*n+i,3*n+j]
        for tet in pattern:
            ids=[cell[k] for k in tet];p=vertices[ids]
            if np.linalg.det((p[1:]-p[0]).T)<0:ids[1],ids[2]=ids[2],ids[1]
            tets.append(ids)
    return vertices,np.asarray(tets,dtype=np.int32),n

def volumes(q,tets):
    p=q[tets];return np.linalg.det(np.stack([p[:,1]-p[:,0],p[:,2]-p[:,0],p[:,3]-p[:,0]],axis=-1))/6

class Simulation:
    def __init__(self,args):
        self.args=args;wp.init();wp.set_device(args.device)
        self.rest,self.tets,self.n=flex_mesh(args.samples)
        b=newton.ModelBuilder(gravity=(0,0,0))
        b.default_particle_radius=.000025
        b.add_soft_mesh(pos=(0,0,0),rot=wp.quat_identity(),scale=1.,vel=(0,0,0),vertices=self.rest.tolist(),indices=self.tets.ravel().tolist(),density=1200.,k_mu=args.young/(2*(1+.3)),k_lambda=args.young*.3/((1+.3)*(1-2*.3)),k_damp=.001,particle_radius=.000025,validate_mesh=True,label='flexspline')
        self.surface=np.asarray(b.tri_indices,dtype=np.int32)
        self.circular=ring_mesh(60*12,[(-.003,lambda t:np.full_like(t,.034)),(.003,lambda t:np.full_like(t,.034)),(.003,lambda t:.02675+.0009*(1+np.cos(60*t))/2),(-.003,lambda t:.02675+.0009*(1+np.cos(60*t))/2)])
        self.cam_minor=getattr(args,"cam_minor",.0228)
        if not .020 <= self.cam_minor <= .02535:
            raise ValueError("cam_minor must be between 0.020 and 0.02535 metres")
        self.cam=ring_mesh(240,[(-.003,lambda t:ellipse(.02535,self.cam_minor,t)),(.003,lambda t:ellipse(.02535,self.cam_minor,t)),(.003,lambda t:np.full_like(t,.004)),(-.003,lambda t:np.full_like(t,.004))])
        cfg=newton.ModelBuilder.ShapeConfig(density=0,ke=args.contact_ke,kd=.005,mu=args.friction,margin=0,gap=.00015,has_shape_collision=False)
        ring_cfg=cfg.copy();ring_cfg.has_particle_collision=not args.no_ring_contact
        self.ring_shape=b.add_shape_mesh(-1,mesh=newton.Mesh(*[self.circular[0],self.circular[1].ravel()]),cfg=ring_cfg,label='fixed circular spline')
        self.cam_body=b.add_body(is_kinematic=True,label='motor driven wave generator')
        self.cam_shape=b.add_shape_mesh(self.cam_body,mesh=newton.Mesh(self.cam[0],self.cam[1].ravel()),cfg=cfg,label='elliptical bearing envelope')
        b.color()
        self.model=b.finalize(device=args.device)
        self.model.soft_contact_ke=args.contact_ke;self.model.soft_contact_kd=.005;self.model.soft_contact_mu=args.friction
        self.pipeline=newton.CollisionPipeline(self.model,soft_contact_margin=.00015,soft_contact_max=len(self.rest)*4,include_static_kinematic_pairs=False)
        self.contacts=self.pipeline.contacts()
        self.solver=SolverVBD(self.model,iterations=args.iterations,particle_enable_tile_solve=False,particle_enable_self_contact=False,rigid_body_particle_contact_buffer_size=len(self.rest)*2,friction_epsilon=.0001)
        self.a=self.model.state();self.b=self.model.state();self.control=self.model.control()
        # Initial assembly preload only; all subsequent flex motion is integrated by VBD.
        initial=self.rest.copy();initial[:,0]*=1.033;initial[:,1]*=.967
        self.a.particle_q.assign(initial);self.b.particle_q.assign(initial)
        self.v0=volumes(self.rest,self.tets)
        self.t=0.;self.angle=0.;self.frames=[];self.rows=[]
        self.capture()
    def motor_angle(self,t):
        x=max(0.,t-self.args.settle)
        # Smooth velocity ramp, then constant speed.
        r=.25
        return self.args.speed*(x-r*(1-math.exp(-x/r)))
    def step(self):
        dt=1/self.args.fps/self.args.substeps
        for _ in range(self.args.substeps):
            self.t+=dt;self.angle=self.motor_angle(self.t)
            pose=np.array([[0,0,0,0,0,math.sin(self.angle/2),math.cos(self.angle/2)]],dtype=np.float32)
            self.a.body_q.assign(pose)
            self.a.body_qd.assign(np.array([[0,0,0,0,0,self.args.speed*(1-math.exp(-max(0.,self.t-self.args.settle)/.25))]],dtype=np.float32))
            self.a.clear_forces()
            self.pipeline.collide(self.a,self.contacts)
            self.solver.step(self.a,self.b,self.control,self.contacts,dt)
            self.a,self.b=self.b,self.a
        self.capture()
    def capture(self):
        q=self.a.particle_q.numpy()
        if not np.isfinite(q).all():raise RuntimeError('Non-finite particle state')
        vr=volumes(q,self.tets)/self.v0
        if vr.min()<=0:raise RuntimeError(f'Inverted tet at {self.t:.4f}s: {vr.min()}')
        # Mean material angular displacement; shape deformation cancels over full circumference.
        z=(q[:,0]+1j*q[:,1])/(self.rest[:,0]+1j*self.rest[:,1])
        output=float(np.angle(np.mean(z/np.abs(z))))
        self.pipeline.collide(self.a,self.contacts)
        count=int(self.contacts.soft_contact_count.numpy()[0]);ids=self.contacts.soft_contact_particle.numpy()[:count];sh=self.contacts.soft_contact_shape.numpy()[:count]
        cp=self.contacts.soft_contact_body_pos.numpy()[:count].copy();norm=self.contacts.soft_contact_normal.numpy()[:count]
        m=sh==self.cam_shape;c,s=math.cos(self.angle),math.sin(self.angle)
        xy=cp[m,:2].copy();cp[m,0]=c*xy[:,0]-s*xy[:,1];cp[m,1]=s*xy[:,0]+c*xy[:,1]
        depth=.000025-np.sum((q[ids]-cp)*norm,axis=1)
        active=depth>0
        pressure=np.zeros(len(q));np.maximum.at(pressure,ids,np.maximum(depth,0)*self.args.contact_ke)
        row={'time_s':self.t,'input_rad':self.angle,'output_rad':output,'ring_contacts':int(np.sum(active & ~m)),'cam_contacts':int(np.sum(active & m)),'max_penetration_mm':float(max(0,depth.max(initial=0))*1000),'min_volume_ratio':float(vr.min()),'max_volume_ratio':float(vr.max()),'max_contact_penalty_N':float(pressure.max()),'center_drift_mm':float(np.linalg.norm(q.mean(axis=0)[:2])*1000)}
        self.rows.append(row)
        self.frames.append({'t':round(self.t,5),'angle':round(self.angle,6),'q':np.round(q*1000,4).tolist(),'force':np.round(pressure,5).tolist(),'metrics':row})

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--no-ring-contact',action='store_true',help='Validation ablation: disable outer gear contact')
    p.add_argument('--device',default='cpu');p.add_argument('--duration',type=float,default=3.)
    p.add_argument('--fps',type=int,default=30);p.add_argument('--substeps',type=int,default=8);p.add_argument('--iterations',type=int,default=15)
    p.add_argument('--samples',type=int,default=4);p.add_argument('--young',type=float,default=2e6)
    p.add_argument('--contact-ke',type=float,default=1e5);p.add_argument('--friction',type=float,default=.05)
    p.add_argument('--cam-minor',type=float,default=.0228,help='Rigid cam minor semiaxis in metres; original demo: 0.02362')
    p.add_argument('--speed',type=float,default=1.);p.add_argument('--settle',type=float,default=.5)
    p.add_argument('--out',type=Path,default=ROOT/'results')
    args=p.parse_args();args.out.mkdir(parents=True,exist_ok=True)
    start=time.time();sim=Simulation(args)
    for i in range(round(args.duration*args.fps)):
        sim.step()
        if (i+1)%args.fps==0:print(json.dumps(sim.rows[-1]),flush=True)
    config=vars(args).copy();config['out']=str(config['out'])
    payload={'engine':f'Newton {newton.__version__}','solver':'VBD','config':config,'surface':sim.surface.tolist(),'circular':{'v':(sim.circular[0]*1000).round(4).tolist(),'f':sim.circular[1].tolist()},'cam':{'v':(sim.cam[0]*1000).round(4).tolist(),'f':sim.cam[1].tolist()},'frames':sim.frames}
    (args.out/'trajectory.json').write_text(json.dumps(payload,separators=(',',':')))
    with (args.out/'metrics.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=sim.rows[0]);w.writeheader();w.writerows(sim.rows)
    summary={'engine':payload['engine'],'solver':'VBD','wall_seconds':time.time()-start,'particles':len(sim.rest),'tetrahedra':len(sim.tets),'max_penetration_mm':max(r['max_penetration_mm'] for r in sim.rows),'min_volume_ratio':min(r['min_volume_ratio'] for r in sim.rows),'peak_ring_contacts':max(r['ring_contacts'] for r in sim.rows),'peak_cam_contacts':max(r['cam_contacts'] for r in sim.rows),'final':sim.rows[-1]}
    settled=[r for r in sim.rows if r['time_s']>=args.settle]
    summary['steady_max_penetration_mm']=max(r['max_penetration_mm'] for r in settled) if settled else None
    if len(settled)>2 and settled[-1]['input_rad']>0:
        summary['measured_output_per_input']=float(np.polyfit([r['input_rad'] for r in settled],[r['output_rad'] for r in settled],1)[0])
        summary['ideal_output_per_input']=-2/58
    (args.out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
