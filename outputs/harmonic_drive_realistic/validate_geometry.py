"""CPU checks independent of the availability of a CUDA device."""
import json
from pathlib import Path
from collections import Counter
import numpy as np
from geometry import Design,cup_mesh,bearing_mesh,bearing_cam_mesh,gear_mesh,volumes,involute_radius

def boundary_faces(tets):
    # Unoriented incidence is sufficient for topology validation.
    f=np.concatenate([tets[:,[0,1,2]],tets[:,[0,1,3]],tets[:,[0,2,3]],tets[:,[1,2,3]]])
    unique,count=np.unique(np.sort(f,axis=1),axis=0,return_counts=True)
    assert np.all(count<=2),'non-manifold tetrahedron interface'
    boundary=unique[count==1]
    edges=np.concatenate([boundary[:,[0,1]],boundary[:,[1,2]],boundary[:,[2,0]]])
    _,ec=np.unique(np.sort(edges,axis=1),axis=0,return_counts=True)
    assert np.all(ec==2),'open or non-manifold boundary'
    return boundary

def mesh_check(v,f):
    edge={}
    for tri in f:
        for a,b in zip(tri,np.roll(tri,-1)):edge.setdefault(tuple(sorted((int(a),int(b)))),[]).append((int(a),int(b)))
    assert all(len(e)==2 and e[0]==e[1][::-1] for e in edge.values())
    p=v[f];area=np.linalg.norm(np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]),axis=1)/2
    assert area.min()>0
    signed=np.einsum('ij,ij->i',p[:,0],np.cross(p[:,1],p[:,2])).sum()/6
    assert signed>0
    return {'vertices':len(v),'triangles':len(f),'closed_consistent_winding':True,'volume_mm3':float(signed*1e9)}

def validate(d=Design(),samples=8,layers=2,axial_refine=1):
    cup,ct,hub,n=cup_mesh(d,samples,layers,axial_refine)
    race,rt,_=bearing_mesh(d,n,layers,2*axial_refine)
    stats={}
    for name,v,t in [('cup',cup,ct),('bearing_outer_race',race,rt)]:
        vol=volumes(v,t);assert vol.min()>0
        boundary=boundary_faces(t)
        assert len(np.unique(t))==len(v)
        stats[name]={'vertices':len(v),'tetrahedra':len(t),'surface_triangles':len(boundary),'min_tet_volume_m3':float(vol.min()),'volume_mm3':float(vol.sum()*1e9),'mass_g':float(vol.sum()*d.density*1000)}
    stats['circular_spline']=mesh_check(*gear_mesh(d,60*24,0,d.face_width))
    stats['cam']=mesh_check(*bearing_cam_mesh(d,960))
    # Exact torque distribution and ideal bearing projection invariants.
    q=cup[hub];forces=d.output_torque*np.c_[-q[:,1],q[:,0],np.zeros(len(q))]/(len(q)*d.hub_radius**2)
    torque=np.cross(q,forces).sum(axis=0)
    assert np.allclose(torque,[0,0,d.output_torque],atol=1e-12)
    # Two independent profiles actually contain their intended tooth counts.
    for z,internal in [(58,False),(60,True)]:
        theta=(np.arange(z*128)+.19)*2*np.pi/(z*128)
        r=involute_radius(theta,z,d.module,internal,d.backlash,d.pressure_angle)
        rr=(r-r.min())/(r.max()-r.min())
        crossings=np.sum((rr>.5)&(np.roll(rr,1)<=.5));assert crossings==z,(crossings,z)
    stats['load_distribution_torque_Nm']=torque.tolist()
    stats['passed']=True
    stats['scope']='Mesh topology, signed volumes, tooth count, load distribution only. CUDA execution and mechanical convergence are not validated.'
    return stats

if __name__=='__main__':
    print(json.dumps(validate(),indent=2))
