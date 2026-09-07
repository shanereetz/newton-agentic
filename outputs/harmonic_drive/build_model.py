"""Generate a simplified static harmonic-drive model. Python 3, no dependencies."""
import math,json,struct
from pathlib import Path
O=Path(__file__).resolve().parent
TAU=2*math.pi
# Overall diameter 68 mm. Illustrative teeth, not conjugate engineering profiles.
def ellipse(a,b,t): return a*b/math.sqrt((b*math.cos(t))**2+(a*math.sin(t))**2)
def ring(n,z,fn): return [(round(fn(TAU*i/n)*math.cos(TAU*i/n),5),round(fn(TAU*i/n)*math.sin(TAU*i/n),5),z) for i in range(n)]
def loft(n,sections):
 v=sum([ring(n,z,fn) for z,fn in sections],[]);f=[]
 for k in range(len(sections)):
  l=(k+1)%len(sections)
  for i in range(n):
   j=(i+1)%n;a=k*n+i;b=k*n+j;c=l*n+j;d=l*n+i
   f.extend([(a,b,c),(a,c,d)])
 return v,f
const=lambda r:lambda t:r
parts=[('Circular spline',[.22,.48,.68,1],loft(480,[(17,const(34)),(25,const(34)),(25,lambda t:27+.9*(1+math.cos(60*t))/2),(17,lambda t:27+.9*(1+math.cos(60*t))/2)])),
 ('Flexspline',[.88,.48,.19,1],loft(464,[(0,const(24)),(3,const(24)),(17,lambda t:ellipse(26.8,24.6,t)),(17,lambda t:ellipse(26.8,24.6,t)+.08+.57*(1+math.cos(58*t))/2),(24,lambda t:ellipse(26.8,24.6,t)+.08+.57*(1+math.cos(58*t))/2),(24,lambda t:ellipse(25.4,23.2,t)),(3,const(22.6)),(3,const(5)),(0,const(5))])),
 ('Wave generator',[.30,.67,.53,1],loft(160,[(17.5,lambda t:ellipse(25.1,22.9,t)),(23.5,lambda t:ellipse(25.1,22.9,t)),(23.5,const(4)),(17.5,const(4))]))]
def normal(a,b,c):
 u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)];w=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];d=math.sqrt(sum(x*x for x in w));assert d>1e-10;return [x/d for x in w]
def stl(path,items):
 faces=[]
 for _,_,(v,f) in items:
  for face in f:
   a,b,c=[v[i] for i in face];faces.append(struct.pack('<12fH',*normal(a,b,c),*a,*b,*c,0))
 path.write_bytes(b'Simplified harmonic drive - mm'.ljust(80,b' ')+struct.pack('<I',len(faces))+b''.join(faces))
for name,col,(v,f) in parts:
 edges={};volume=0
 for face in f:
  a,b,c=[v[i] for i in face];normal(a,b,c)
  volume+=(a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6
  for a,b in zip(face,face[1:]+face[:1]):edges.setdefault(tuple(sorted((a,b))),[]).append((a,b))
 assert all(len(e)==2 and e[0]==e[1][::-1] for e in edges.values())
 assert volume>0
 stl(O/(name.lower().replace(' ','_')+'.stl'),[(name,col,(v,f))])
 print(name,len(f),'triangles; closed, consistently wound; volume',round(volume,2),'mm3')
stl(O/'harmonic_drive_assembly.stl',parts)
# Color glTF binary, expressed in metres per glTF specification. Z-up CAD mapped to Y-up.
doc={'asset':{'version':'2.0','generator':'Simplified harmonic drive'},'scene':0,'scenes':[{'nodes':[0,1,2]}],'nodes':[],'meshes':[],'materials':[],'buffers':[],'bufferViews':[],'accessors':[]};blob=bytearray()
for k,(name,col,(v,f)) in enumerate(parts):
 pos=[];norm=[]
 for face in f:
  a,b,c=[v[i] for i in face];nx,ny,nz=normal(a,b,c)
  for x,y,z in (a,b,c):pos.extend([x/1000,z/1000,-y/1000]);norm.extend([nx,nz,-ny])
 for values in (pos,norm):
  offset=len(blob);blob.extend(struct.pack('<%sf'%len(values),*values));doc['bufferViews'].append({'buffer':0,'byteOffset':offset,'byteLength':len(values)*4,'target':34962});acc={'bufferView':len(doc['bufferViews'])-1,'componentType':5126,'count':len(values)//3,'type':'VEC3'}
  if values is pos:acc.update(min=[min(values[i::3]) for i in range(3)],max=[max(values[i::3]) for i in range(3)])
  doc['accessors'].append(acc)
 doc['materials'].append({'name':name,'pbrMetallicRoughness':{'baseColorFactor':col,'metallicFactor':.15,'roughnessFactor':.4}})
 doc['meshes'].append({'name':name,'primitives':[{'attributes':{'POSITION':2*k,'NORMAL':2*k+1},'material':k}]});doc['nodes'].append({'name':name,'mesh':k})
doc['buffers']=[{'byteLength':len(blob)}]
def glb(path):
 j=json.dumps(doc,separators=(',',':')).encode();j+=b' '*((-len(j))%4);b=bytes(blob);b+=b'\0'*((-len(b))%4);path.write_bytes(struct.pack('<III',0x46546c67,2,28+len(j)+len(b))+struct.pack('<II',len(j),0x4e4f534a)+j+struct.pack('<II',len(b),0x004e4942)+b)
glb(O/'harmonic_drive.glb')
for i,z in enumerate([.023,0,.047]):doc['nodes'][i]['translation']=[0,z,0]
glb(O/'harmonic_drive_exploded.glb')
# Small inline preview dataset, actual geometry, reduced to triangle coordinates.
Path('work').mkdir(exist_ok=True)
Path('work/harmonic_mesh.json').write_text(json.dumps([{'name':name,'v':v,'f':f} for name,_,(v,f) in parts],separators=(',',':')))
