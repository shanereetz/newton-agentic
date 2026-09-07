"""Parameterized involute-tooth steel reference design. Geometry in SI metres."""
from dataclasses import dataclass, asdict
import numpy as np

@dataclass
class Design:
    module: float = .0009
    flex_teeth: int = 58
    circular_teeth: int = 60
    pressure_angle: float = 20.
    backlash: float = .00004
    wall: float = .0004
    cup_depth: float = .025
    face_width: float = .004
    hub_radius: float = .008
    outside_radius: float = .034
    young: float = 206e9
    poisson: float = .3
    density: float = 7850.
    # Explicit design inputs, NOT a commercial gearbox calibration.
    friction_teeth: float = .05
    friction_bearing: float = .002
    normal_penalty: float = 2e14  # N/m^3, traction per closure
    output_torque: float = .1  # N m

def involute_radius(theta,z,module,internal=False,backlash=0.,pressure_angle=20.):
    rp=module*z/2;rb=rp*np.cos(np.deg2rad(pressure_angle))
    lo=rp-module if internal else rp-1.25*module
    hi=rp+1.25*module if internal else rp+module
    alpha=np.deg2rad(pressure_angle)
    half=np.pi/(2*z)+(backlash/(2*rp) if internal else -backlash/(2*rp))
    inv0=np.tan(alpha)-alpha
    angle=np.abs((theta+np.pi/z)%(2*np.pi/z)-np.pi/z)
    a=np.full_like(angle,lo);b=np.full_like(angle,hi)
    for _ in range(45):
        r=(a+b)/2
        t=np.arccos(np.clip(rb/r,-1,1));bound=half+inv0-(np.tan(t)-t)
        # At tooth/space centre, radius is the largest.
        a=np.where(angle<bound,r,a);b=np.where(angle>=bound,r,b)
    return (a+b)/2

def gear_mesh(d,n,z0,z1,internal=True):
    theta=np.arange(n)*2*np.pi/n
    r=involute_radius(theta,d.circular_teeth if internal else d.flex_teeth,d.module,internal,d.backlash,d.pressure_angle)
    outer=np.full(n,d.outside_radius)
    rings=[(z0,outer),(z1,outer),(z1,r),(z0,r)]
    v=np.concatenate([np.c_[rr*np.cos(theta),rr*np.sin(theta),np.full(n,zz)] for zz,rr in rings])
    f=[]
    for k in range(4):
        for i in range(n):
            j=(i+1)%n;a=k*n+i;b=k*n+j;c=((k+1)%4)*n+j;e=((k+1)%4)*n+i
            f.extend([[a,b,c],[a,c,e]])
    return v.astype(np.float32),np.asarray(f,np.int32)

def cam_mesh(d,n=240):
    inner=d.module*d.flex_teeth/2-1.25*d.module-d.wall
    delta=d.module
    # Ellipse axes chosen with approximately conserved neutral circumference.
    a=inner+delta;b=inner-delta-delta*delta/(2*inner)
    theta=np.arange(n)*2*np.pi/n
    # Lead-in chamfers avoid an abrupt axial edge against the cup.
    rings=[(-.001,.96),(-.0005,1.),(d.face_width-.0005,1.),(d.face_width,.96)]
    v=[np.c_[a*s*np.cos(theta),b*s*np.sin(theta),np.full(n,z)] for z,s in rings]
    v.extend([np.c_[.004*np.cos(theta),.004*np.sin(theta),np.full(n,z)] for z in [d.face_width,-.001]])
    v=np.concatenate(v);f=[]
    for k in range(6):
        for i in range(n):
            j=(i+1)%n;a0=k*n+i;b0=k*n+j;c=((k+1)%6)*n+j;e=((k+1)%6)*n+i
            f.extend([[a0,b0,c],[a0,c,e]])
    return v.astype(np.float32),np.asarray(f,np.int32),inner

def cup_mesh(d,samples=4,layers=2,axial_refine=1):
    n=d.flex_teeth*samples;t=np.arange(n)*2*np.pi/n
    root=d.module*d.flex_teeth/2-1.25*d.module
    # Cross-sectional strip: hub diaphragm, rounded heel, thin cup wall, toothed rim.
    stations=[(d.hub_radius,-d.cup_depth),(root*.65,-d.cup_depth),
              (root-.001,-d.cup_depth),(root-d.wall/2,-d.cup_depth+.001),
              (root-d.wall/2,-.017),(root-d.wall/2,-.009),
              (root-d.wall/2,-.001),(root-d.wall/2,0),(root-d.wall/2,d.face_width/2),(root-d.wall/2,d.face_width)]
    if axial_refine>1:
        dense=[]
        for a,b in zip(stations[:-1],stations[1:]):
            for k in range(axial_refine):dense.append(tuple(np.array(a)+(np.array(b)-a)*k/axial_refine))
        stations=dense+[stations[-1]]
    st=np.asarray(stations);v=[]
    for j,(r,z) in enumerate(st):
        tangent=st[min(j+1,len(st)-1)]-st[max(0,j-1)]
        tangent/=np.linalg.norm(tangent);normal=np.array([tangent[1],-tangent[0]])
        thickness=.0012 if j<2*axial_refine else d.wall
        for k in range(layers+1):
            offset=(k/layers-.5)*thickness
            rr=np.full(n,r+normal[0]*offset);zz=np.full(n,z+normal[1]*offset)
            if z>=0:
                tooth=involute_radius(t,d.flex_teeth,d.module,False,d.backlash,d.pressure_angle)-root
                rr+=tooth*(k/layers)
            v.extend(np.c_[rr*np.cos(t),rr*np.sin(t),zz])
    v=np.asarray(v,np.float64);tets=[]
    def idx(s,k,i):return (s*(layers+1)+k)*n+i%n
    pat=[(0,1,3,7),(0,3,2,7),(0,2,6,7),(0,6,4,7),(0,4,5,7),(0,5,1,7)]
    for j in range(len(st)-1):
        for k in range(layers):
            for i in range(n):
                cell=[idx(j,k,i),idx(j,k,i+1),idx(j,k+1,i),idx(j,k+1,i+1),idx(j+1,k,i),idx(j+1,k,i+1),idx(j+1,k+1,i),idx(j+1,k+1,i+1)]
                for tet in pat:
                    ids=[cell[x] for x in tet];p=v[ids]
                    if np.linalg.det((p[1:]-p[0]).T)<0:ids[1],ids[2]=ids[2],ids[1]
                    tets.append(ids)
    return v,np.asarray(tets,np.int32),np.arange((layers+1)*n,dtype=np.int32),n

def volumes(v,t):
    p=v[t];return np.linalg.det(np.stack([p[:,1]-p[:,0],p[:,2]-p[:,0],p[:,3]-p[:,0]],axis=-1))/6

def bearing_mesh(d,n,layers=2,axial_layers=2):
    """Elastic outer race; rolling elements replaced by a low-friction cam envelope."""
    inner_cup=d.module*d.flex_teeth/2-1.25*d.module-d.wall
    outer=inner_cup-.00001;inner=outer-.0003
    theta=np.arange(n)*2*np.pi/n;v=[]
    for z in np.linspace(.00015,d.face_width-.00015,axial_layers+1):
        for r in np.linspace(inner,outer,layers+1):v.extend(np.c_[r*np.cos(theta),r*np.sin(theta),np.full(n,z)])
    v=np.asarray(v);t=[]
    def idx(a,r,i):return (a*(layers+1)+r)*n+i%n
    pat=[(0,1,3,7),(0,3,2,7),(0,2,6,7),(0,6,4,7),(0,4,5,7),(0,5,1,7)]
    for a in range(axial_layers):
        for r in range(layers):
            for i in range(n):
                c=[idx(a,r,i),idx(a,r,i+1),idx(a,r+1,i),idx(a,r+1,i+1),idx(a+1,r,i),idx(a+1,r,i+1),idx(a+1,r+1,i),idx(a+1,r+1,i+1)]
                for tet in pat:
                    ids=[c[j] for j in tet];p=v[ids]
                    if np.linalg.det((p[1:]-p[0]).T)<0:ids[1],ids[2]=ids[2],ids[1]
                    t.append(ids)
    return v,np.asarray(t,np.int32),inner

def bearing_cam_mesh(d,n=480):
    inner=d.module*d.flex_teeth/2-1.25*d.module-d.wall-.00001-.0003-.000005
    delta=d.module;a=inner+delta;b=inner-delta-delta*delta/(2*inner)
    theta=np.arange(n)*2*np.pi/n
    # A 4 mm axial lead-in gradually seats the cam, avoiding a penetrated initial state.
    rings=[(-.004,.94),(0.,1.),(d.face_width+.0002,1.)]
    arrays=[np.c_[a*s*np.cos(theta),b*s*np.sin(theta),np.full(n,z)] for z,s in rings]
    arrays.extend([np.c_[.004*np.cos(theta),.004*np.sin(theta),np.full(n,z)] for z in [d.face_width+.0002,-.004]])
    v=np.concatenate(arrays);f=[]
    for k in range(5):
        for i in range(n):
            j=(i+1)%n;a0=k*n+i;b0=k*n+j;c=((k+1)%5)*n+j;e=((k+1)%5)*n+i
            f.extend([[a0,b0,c],[a0,c,e]])
    return v.astype(np.float32),np.asarray(f,np.int32)
