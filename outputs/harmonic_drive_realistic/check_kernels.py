"""Exercise the CUDA experiment's Warp kernels on CPU without pretending to test SDF/CUDA."""
import numpy as np
import warp as wp
from realistic import drive,load_hub,support,constrain_velocity,participation,pressure_weights

def check():
    wp.init();wp.set_device('cpu')
    poses=wp.zeros(2,dtype=wp.transform);vel=wp.zeros(2,dtype=wp.spatial_vector)
    wp.launch(drive,dim=1,inputs=[0.,.1,poses,vel]);p=poses.numpy()
    assert np.allclose(p[:,2],[.012,.010])
    wp.launch(drive,dim=1,inputs=[5.,.1,poses,vel]);assert np.allclose(poses.numpy()[:,:3],0)
    wp.launch(drive,dim=1,inputs=[6.,.1,poses,vel]);assert vel.numpy()[1,5]>0
    rest=np.array([[.008,0,0],[0,.008,0],[.024,0,.004]],np.float32)
    r=wp.array(rest,dtype=wp.vec3);q=wp.array(rest+np.array([.0001,.0002,.0003]),dtype=wp.vec3)
    wp.launch(support,dim=3,inputs=[q,r,2,2]);a=q.numpy()
    assert np.allclose(np.linalg.norm(a[:2,:2],axis=1),.008)
    assert np.allclose(a[:,2],rest[:,2])
    forces=wp.zeros(3,dtype=wp.vec3);ids=wp.array([0,1],dtype=int)
    wp.launch(load_hub,dim=2,inputs=[q,ids,.1,.008,forces])
    assert np.isclose(np.cross(a,forces.numpy()).sum(axis=0)[2],.1,atol=1e-7)
    qd=wp.array(np.ones((3,3)),dtype=wp.vec3)
    wp.launch(constrain_velocity,dim=3,inputs=[q,qd,2,2])
    assert np.allclose(np.sum(a[:2,:2]*qd.numpy()[:2,:2],axis=1),0,atol=1e-8)
    c=wp.array([2],dtype=int);ids=wp.array([[0,1,2],[0,1,2]],dtype=wp.vec3i)
    b=wp.array([[1/3]*3,[1/3]*3],dtype=wp.vec3);sh=wp.array([0,0],dtype=int)
    sums=wp.zeros((2,3),dtype=float);area=wp.array([1.,2.,3.],dtype=float)
    w=wp.zeros(2,dtype=float);k=wp.zeros(2,dtype=float);kd=wp.zeros(2,dtype=float);mu=wp.zeros(2,dtype=float);fr=wp.array([.05,.002],dtype=float)
    wp.launch(participation,dim=2,inputs=[c,ids,b,sh,sums])
    wp.launch(pressure_weights,dim=2,inputs=[c,ids,b,sh,sums,area,10.,fr,w,k,kd,mu])
    assert np.isclose(w.numpy().sum(),6.)
    assert np.allclose(k.numpy(),30.)
    return {'warp_kernel_compilation_cpu':True,'assembly_staging':True,'free_rotation_support':True,'load_torque_distribution':True,'contact_area_not_duplicated':True,'CUDA_full_scene_tested':False}
if __name__=='__main__':
    import json
    print(json.dumps(check(),indent=2))
