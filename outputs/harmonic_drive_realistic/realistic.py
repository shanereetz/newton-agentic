"""CUDA steel reference experiment using Newton 1.5.1 VBD and native SDF contacts.
This branch is a candidate, not a validated commercial harmonic-drive model.
"""
from pathlib import Path
from dataclasses import asdict
import argparse, json, math, csv, time, os
import numpy as np
import warp as wp
import newton
from newton.solvers import SolverVBD
from geometry import Design, cup_mesh, bearing_mesh, bearing_cam_mesh, gear_mesh, volumes

ROOT=Path(__file__).resolve().parent
wp.config.kernel_cache_dir=os.environ.get('WARP_CACHE_PATH',str(ROOT/'.warp_cache'))
newton.use_coord_layout_targets=True

@wp.func
def smooth(x:float):
    u=wp.clamp(x,0.0,1.0)
    return u*u*(3.0-2.0*u)

@wp.kernel
def drive(t:float,speed:float,poses:wp.array[wp.transform],vel:wp.array[wp.spatial_vector]):
    # Cam seats first, circular spline then seats around the deformed cup.
    cam_z=.010*(1.0-smooth(t/2.0))
    ring_z=.012*(1.0-smooth((t-2.5)/2.0))
    run=wp.max(t-5.0,0.0)
    angle=speed*(run-.5*(1.0-wp.exp(-run/.5)))
    omega=speed*(1.0-wp.exp(-run/.5))
    poses[0]=wp.transform(wp.vec3(0.0,0.0,ring_z),wp.quat_identity())
    poses[1]=wp.transform(wp.vec3(0.0,0.0,cam_z),wp.quat_from_axis_angle(wp.vec3(0.0,0.0,1.0),angle))
    u=wp.clamp(t/2.0,0.0,1.0);v=wp.clamp((t-2.5)/2.0,0.0,1.0)
    vel[0]=wp.spatial_vector(0.0,0.0,-.012*6.0*v*(1.0-v)/2.0,0.0,0.0,0.0)
    vel[1]=wp.spatial_vector(0.0,0.0,-.010*6.0*u*(1.0-u)/2.0,0.0,0.0,omega)

@wp.kernel
def load_hub(q:wp.array[wp.vec3],ids:wp.array[int],torque:float,radius:float,forces:wp.array[wp.vec3]):
    i=ids[wp.tid()];p=q[i]
    # Positive Z torque opposes the expected negative output rotation.
    scale=torque/(float(ids.shape[0])*radius*radius)
    forces[i]=wp.vec3(-p[1]*scale,p[0]*scale,0.0)

@wp.kernel
def support(q:wp.array[wp.vec3],rest:wp.array[wp.vec3],hub_count:int,race_start:int):
    i=wp.tid();p=q[i];r=rest[i]
    if i<hub_count:
        # Ideal output bearing: radial and axial support, free circumferential motion.
        rho=wp.sqrt(p[0]*p[0]+p[1]*p[1]);r0=wp.sqrt(r[0]*r[0]+r[1]*r[1])
        if rho>1.e-12:p=wp.vec3(p[0]*r0/rho,p[1]*r0/rho,r[2])
    if i>=race_start:
        # Ideal axial retention of the outer race; radial deformation remains free.
        p=wp.vec3(p[0],p[1],r[2])
    q[i]=p

@wp.kernel
def constrain_velocity(q:wp.array[wp.vec3],qd:wp.array[wp.vec3],hub_count:int,race_start:int):
    i=wp.tid();v=qd[i];p=q[i]
    if i<hub_count:
        rr=p[0]*p[0]+p[1]*p[1]
        radial=(p[0]*v[0]+p[1]*v[1])/wp.max(rr,1.e-20)
        v=wp.vec3(v[0]-radial*p[0],v[1]-radial*p[1],0.0)
    if i>=race_start:v=wp.vec3(v[0],v[1],0.0)
    qd[i]=v

@wp.kernel
def participation(count:wp.array[int],ids:wp.array[wp.vec3i],bary:wp.array[wp.vec3],shapes:wp.array[int],totals:wp.array2d[float]):
    c=wp.tid()
    if c<count[0]:
        for j in range(3):
            i=ids[c][j]
            if i>=0:wp.atomic_add(totals,shapes[c],i,bary[c][j])

@wp.kernel
def pressure_weights(count:wp.array[int],ids:wp.array[wp.vec3i],bary:wp.array[wp.vec3],shapes:wp.array[int],totals:wp.array2d[float],area:wp.array[float],penalty:float,friction:wp.array[float],weights:wp.array[float],ke:wp.array[float],kd:wp.array[float],mu:wp.array[float]):
    c=wp.tid()
    if c<count[0]:
        a=0.0;s=shapes[c]
        for j in range(3):
            i=ids[c][j]
            if i>=0:a+=bary[c][j]*area[i]/wp.max(totals[s,i],1.e-12)
        weights[c]=a;ke[c]=penalty*a;kd[c]=0.0;mu[c]=friction[s]

class SupportedVBD(SolverVBD):
    """Newton VBD with ideal-bearing constraints and tributary-area contact penalties.
    Internal hooks are pinned to Newton 1.5.1 and covered by API preflight checks.
    """
    def __init__(self,model,rest,hub_count,race_start,nodal_area,capacity,d,**kw):
        self.rest_positions=wp.array(rest,dtype=wp.vec3,device=model.device)
        self.hub_count=hub_count;self.race_start=race_start
        self.area=wp.array(nodal_area,dtype=float,device=model.device)
        self.participation=wp.zeros((2,len(rest)),dtype=float,device=model.device)
        self.contact_area=wp.zeros(capacity,dtype=float,device=model.device)
        self.physical_penalty=d.normal_penalty
        self.contact_friction=wp.array([d.friction_teeth,d.friction_bearing],dtype=float,device=model.device)
        super().__init__(model,**kw)
    def _initialize_rigid_bodies(self,state_in,control,contacts,dt,refresh):
        super()._initialize_rigid_bodies(state_in,control,contacts,dt,refresh)
        self.participation.zero_()
        base=[contacts.soft_contact_count,contacts.soft_contact_indices,contacts.soft_contact_barycentric,contacts.soft_contact_shape]
        wp.launch(participation,dim=contacts.soft_contact_max,inputs=base+[self.participation],device=self.device)
        wp.launch(pressure_weights,dim=contacts.soft_contact_max,inputs=base+[self.participation,self.area,self.physical_penalty,self.contact_friction,self.contact_area,self.body_particle_contact_penalty_k,self.body_particle_contact_material_kd,self.body_particle_contact_material_mu],device=self.device)
    def _solve_particle_iteration(self,state_in,state_out,contacts,dt,iter_num):
        super()._solve_particle_iteration(state_in,state_out,contacts,dt,iter_num)
        wp.launch(support,dim=self.model.particle_count,inputs=[state_out.particle_q,self.rest_positions,self.hub_count,self.race_start],device=self.device)

class Experiment:
    def __init__(self,args):
        self.args=args;self.d=Design(**json.loads(args.design.read_text()))
        d=self.d
        if newton.__version__!='1.5.1':raise RuntimeError('This experiment is pinned to Newton 1.5.1.')
        wp.init()
        if not wp.is_cuda_available():
            raise RuntimeError('CUDA is required for the native mesh SDF/full-surface experiment. No CPU contact fallback is used. Run --geometry-only on this machine.')
        wp.set_device(args.device)
        self.cup,self.cup_tets,self.hub,self.n=cup_mesh(d,args.samples,args.layers,args.axial_refine)
        race,rt,inner=bearing_mesh(d,self.n,args.layers,2*args.axial_refine)
        self.race_start=len(self.cup)
        self.rest=np.vstack([self.cup,race]);self.tets=np.vstack([self.cup_tets,rt+len(self.cup)])
        self.v0=volumes(self.rest,self.tets)
        if np.min(self.v0)<=0:raise ValueError('Non-positive reference tetrahedron')
        p=self.rest[self.tets]
        self.Dm_inv=np.linalg.inv(np.stack([p[:,1]-p[:,0],p[:,2]-p[:,0],p[:,3]-p[:,0]],axis=-1))
        builder=newton.ModelBuilder(gravity=(0,0,0))
        mu=d.young/(2*(1+d.poisson));lam=d.young*d.poisson/((1+d.poisson)*(1-2*d.poisson))
        self.lame=(mu,lam)
        builder.add_soft_mesh(pos=(0,0,0),rot=wp.quat_identity(),scale=1,vel=(0,0,0),vertices=self.rest.tolist(),indices=self.tets.ravel().tolist(),density=d.density,k_mu=mu,k_lambda=lam,k_damp=args.damping,particle_radius=0.,validate_mesh=True,label='steel cup and flexible bearing race')
        self.surface=np.asarray(builder.tri_indices,np.int32)
        p=self.rest[self.surface];fa=np.linalg.norm(np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]),axis=1)/2
        nodal_area=np.zeros(len(self.rest))
        for j in range(3):np.add.at(nodal_area,self.surface[:,j],fa/3)
        self.nodal_area=nodal_area
        # Particle-particle contact stiffness is a nodal penalty. Scale with contact-region area.
        region=(self.rest[:,2]>=0)
        self.self_ke=d.normal_penalty*float(np.median(nodal_area[region]))
        self.rigid_geometry=[gear_mesh(d,d.circular_teeth*args.rigid_samples,0,d.face_width),bearing_cam_mesh(d,args.rigid_samples*40)]
        for k,(v,f) in enumerate(self.rigid_geometry):
            body=builder.add_body(is_kinematic=True,label=['circular spline assembly fixture','wave-generator motor'][k])
            mesh=newton.Mesh(v,f.ravel(),compute_inertia=False)
            print(f'Building CUDA SDF {k}: target voxel {args.voxel*1e6:.1f} um',flush=True)
            mesh.build_sdf(device=args.device,target_voxel_size=args.voxel,narrow_band_range=(-4*args.voxel,4*args.voxel),margin=4*args.voxel,texture_format='float32',cache_dir=str(args.cache/'sdf'))
            cfg=newton.ModelBuilder.ShapeConfig(density=0,ke=self.self_ke,kd=0,mu=[d.friction_teeth,d.friction_bearing][k],margin=0,gap=args.contact_gap,has_shape_collision=False,has_particle_collision=not(args.no_ring_contact and k==0))
            builder.add_shape_mesh(body,mesh=mesh,cfg=cfg,color=[(.25,.48,.70),(.27,.70,.50)][k],label=['circular spline','cam and rolling-support envelope'][k])
        builder.color();self.model=builder.finalize(device=args.device)
        self.model.soft_contact_ke=self.self_ke;self.model.soft_contact_kd=0.;self.model.soft_contact_mu=d.friction_bearing
        self.pipeline=newton.CollisionPipeline(self.model,enable_rigid_soft_full_surface_contact=True,soft_contact_margin=args.contact_gap,include_static_kinematic_pairs=False)
        self.contacts=self.pipeline.contacts()
        self.solver=SupportedVBD(self.model,self.rest,len(self.hub),self.race_start,nodal_area,self.contacts.soft_contact_max,d,iterations=args.iterations,particle_enable_tile_solve=False,particle_enable_self_contact=True,particle_self_contact_radius=args.self_radius,particle_self_contact_margin=args.contact_gap,particle_rest_shape_contact_exclusion_radius=0.,particle_topological_contact_filter_threshold=2,particle_vertex_contact_buffer_size=128,particle_edge_contact_buffer_size=128,rigid_body_particle_contact_buffer_size=max(1024,len(self.rest)*4),friction_epsilon=1.e-5)
        self.a=self.model.state();self.b=self.model.state();self.control=self.model.control();self.hub_ids=wp.array(self.hub,dtype=int,device=args.device)
        self.t=0.;self.rows=[];self.frames=[];self.last_min_J=1.;self.last_max_strain=0.
        self.set_drive();self.pipeline.collide(self.a,self.contacts)
        print(f'CUDA model ready: {len(self.rest)} particles, {len(self.tets)} tets, native full-surface contacts, physical E={d.young/1e9:g} GPa',flush=True)
        self.record()
    def set_drive(self):
        wp.launch(drive,dim=1,inputs=[self.t,self.args.speed,self.a.body_q,self.a.body_qd],device=self.model.device)
    def substep(self):
        self.t+=self.args.dt;self.set_drive();self.a.clear_forces()
        load=self.d.output_torque*min(max((self.t-4.5)/.5,0),1)
        wp.launch(load_hub,dim=len(self.hub),inputs=[self.a.particle_q,self.hub_ids,load,self.d.hub_radius,self.a.particle_f],device=self.model.device)
        self.pipeline.collide(self.a,self.contacts)
        self.solver.step(self.a,self.b,self.control,self.contacts,self.args.dt)
        wp.launch(constrain_velocity,dim=len(self.rest),inputs=[self.b.particle_q,self.b.particle_qd,len(self.hub),self.race_start],device=self.model.device)
        self.a,self.b=self.b,self.a
    def advance(self,seconds):
        for _ in range(max(1,round(seconds/self.args.dt))):self.substep()
        return self.record()
    def record(self):
        q=self.a.particle_q.numpy();vel=self.a.particle_qd.numpy()
        if not np.isfinite(q).all():raise RuntimeError(f'Nonfinite state at t={self.t}')
        p=q[self.tets];F=np.matmul(np.stack([p[:,1]-p[:,0],p[:,2]-p[:,0],p[:,3]-p[:,0]],axis=-1),self.Dm_inv)
        J=np.linalg.det(F)
        if J.min()<=0:raise RuntimeError(f'Inverted element at t={self.t}; min J={J.min()}')
        mu,lam=self.lame;lam_nh=lam+mu;alpha=1+mu/lam_nh
        # Cauchy stress consistent with Newton's stable Neo-Hookean VBD energy.
        B=np.matmul(F,F.transpose(0,2,1));sigma=mu*B/J[:,None,None]+lam_nh*(J-alpha)[:,None,None]*np.eye(3)
        dev=sigma-np.trace(sigma,axis1=1,axis2=2)[:,None,None]*np.eye(3)/3
        vm=np.sqrt(1.5*np.sum(dev*dev,axis=(1,2)))
        strain=.5*(np.matmul(F.transpose(0,2,1),F)-np.eye(3));max_strain=float(np.linalg.norm(strain,axis=(1,2)).max())
        z=(q[self.hub,0]+1j*q[self.hub,1])/(self.rest[self.hub,0]+1j*self.rest[self.hub,1])
        output=float(np.angle(np.mean(z/np.abs(z))))
        count=int(self.contacts.soft_contact_count.numpy()[0])
        ids=self.contacts.soft_contact_indices.numpy()[:count];bary=self.contacts.soft_contact_barycentric.numpy()[:count];shapes=self.contacts.soft_contact_shape.numpy()[:count]
        cp=self.contacts.soft_contact_body_pos.numpy()[:count];normals=self.contacts.soft_contact_normal.numpy()[:count]
        poses=self.a.body_q.numpy();world=cp.copy()
        for k in [0,1]:
            mask=shapes==k;x=poses[k,3:7];v=cp[mask];u=x[:3]
            world[mask]=v+2*np.cross(u,np.cross(u,v)+x[3]*v)+poses[k,:3]
        soft=np.sum(q[np.maximum(ids,0)]*bary[:,:,None],axis=1)
        depth=np.maximum(0,-np.sum((soft-world)*normals,axis=1))
        area=self.solver.contact_area.numpy()[:count]
        pressure=self.d.normal_penalty*depth
        force=pressure[:,None]*area[:,None]*normals
        torque=np.cross(world,-force)[:,2]
        run=max(self.t-5,0);angle=self.args.speed*(run-.5*(1-math.exp(-run/.5)))
        elastic=np.sum(self.v0*(mu/2*(np.sum(F*F,axis=(1,2))-3)+lam_nh/2*((J-alpha)**2-(1-alpha)**2)))
        row=dict(time_s=self.t,input_rad=angle,output_rad=output,load_Nm=self.d.output_torque*min(max((self.t-4.5)/.5,0),1),min_J=float(J.min()),max_J=float(J.max()),max_green_strain=max_strain,max_von_mises_MPa=float(vm.max()/1e6),max_contact_penetration_um=float(depth.max(initial=0)*1e6),max_normal_penalty_pressure_MPa=float(pressure.max(initial=0)/1e6),ring_active_contacts=int(np.sum((shapes==0)&(depth>0))),cam_active_contacts=int(np.sum((shapes==1)&(depth>0))),face_edge_contact_candidates=int(np.sum(ids[:,1]>=0)),ring_normal_reaction_Nm=float(torque[shapes==0].sum()),cam_normal_reaction_Nm=float(torque[shapes==1].sum()),elastic_energy_J=float(elastic),kinetic_energy_J=float(.5*np.sum(self.model.particle_mass.numpy()[:,None]*vel**2)))
        self.rows.append(row)
        if self.args.record_mesh:self.frames.append({'t':self.t,'q':q.tolist(),'body_q':poses.tolist()})
        return row
    def save(self):
        out=self.args.out;out.mkdir(parents=True,exist_ok=True)
        with (out/'metrics.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=self.rows[0]);w.writeheader();w.writerows(self.rows)
        meta={'engine':newton.__version__,'solver':'SupportedVBD (Newton SolverVBD subclass)','device':str(self.model.device),'design':asdict(self.d),'config':{k:str(v) if isinstance(v,Path) else v for k,v in vars(self.args).items()},'status':'exploratory; engineering accuracy requires convergence and external benchmark','final':self.rows[-1]}
        (out/'run.json').write_text(json.dumps(meta,indent=2))
        if self.frames:(out/'trajectory.json').write_text(json.dumps({'surface':self.surface.tolist(),'rigid':[(v.tolist(),f.tolist()) for v,f in self.rigid_geometry],'frames':self.frames},separators=(',',':')))

def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--design',type=Path,default=ROOT/'design.json');p.add_argument('--device',default='cuda:0')
    p.add_argument('--duration',type=float,default=8.);p.add_argument('--dt',type=float,default=.0005);p.add_argument('--iterations',type=int,default=100)
    p.add_argument('--samples',type=int,default=8);p.add_argument('--layers',type=int,default=2);p.add_argument('--axial-refine',type=int,default=1);p.add_argument('--rigid-samples',type=int,default=24)
    p.add_argument('--voxel',type=float,default=2.5e-5);p.add_argument('--contact-gap',type=float,default=7.5e-5);p.add_argument('--self-radius',type=float,default=2e-6)
    p.add_argument('--damping',type=float,default=0.);p.add_argument('--speed',type=float,default=.1)
    p.add_argument('--no-ring-contact',action='store_true');p.add_argument('--record-mesh',action='store_true');p.add_argument('--viewer',action='store_true');p.add_argument('--geometry-only',action='store_true')
    p.add_argument('--out',type=Path,default=ROOT/'runs/default');p.add_argument('--cache',type=Path,default=ROOT/'.cache')
    return p

def main():
    args=parser().parse_args()
    if args.dt<=0 or args.duration<5.5 or args.samples<4 or args.layers<2 or args.voxel<=0 or args.iterations<1:raise ValueError('Use dt>0, duration>=5.5, samples>=4, layers>=2, voxel>0, iterations>=1')
    if args.geometry_only:
        from validate_geometry import validate
        print(json.dumps(validate(Design(**json.loads(args.design.read_text())),args.samples,args.layers,args.axial_refine),indent=2));return
    sim=Experiment(args)
    if args.viewer:
        from newton.viewer import ViewerGL
        viewer=ViewerGL(width=1400,height=900,vsync=True,paused=False);viewer.set_model(sim.model)
        viewer.renderer.set_title('Newton VBD — CUDA Steel Reference (experimental)');viewer.show_ground=False
        viewer.camera.near=.0005;viewer.camera_speed=.025;viewer.set_camera(wp.vec3(.04,-.09,.07),pitch=-45.,yaw=90.)
        try:
            while viewer.is_running():
                if viewer.should_step() and sim.t<args.duration:sim.advance(1/30)
                viewer.begin_frame(sim.t);viewer.log_state(sim.a)
                for key in ['output_rad','load_Nm','max_von_mises_MPa','ring_active_contacts','face_edge_contact_candidates']:viewer.log_scalar(key,sim.rows[-1][key])
                viewer.end_frame()
        finally:sim.save();viewer.close()
    else:
        try:
            while sim.t<args.duration-args.dt/2:
                row=sim.advance(min(.05,args.duration-sim.t));print(json.dumps(row),flush=True)
        finally:sim.save()
if __name__=='__main__':main()
