import * as THREE from 'three';
import {robotColour} from './colours.js';
import {OrbitControls} from './vendor/three/OrbitControls.js';
export function createScene(container,model,data) {
  const renderer=new THREE.WebGLRenderer({alpha:true,antialias:true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio,2));
  renderer.outputColorSpace=THREE.SRGBColorSpace;
  renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
  container.appendChild(renderer.domElement);
  const scene=new THREE.Scene();
  const camera=new THREE.PerspectiveCamera(37,1,.005,30);camera.up.set(0,0,1);camera.position.set(.32,-.43,.27);
  const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(0,0,.08);
  controls.enableDamping=true;controls.minDistance=.15;controls.maxDistance=3;controls.maxPolarAngle=Math.PI*.49;controls.update();
  scene.add(new THREE.HemisphereLight(0xd6edff,0x344356,2.1));
  const light=new THREE.DirectionalLight(0xffedda,3.2);light.position.set(.4,-.5,1);
  light.castShadow=true;light.shadow.mapSize.set(2048,2048);light.shadow.camera.left=-.4;light.shadow.camera.right=.4;light.shadow.camera.top=.4;light.shadow.camera.bottom=-.4;light.shadow.camera.near=.01;light.shadow.camera.far=3;light.shadow.bias=-.00015;
  scene.add(light,light.target);
  const floor=new THREE.Mesh(new THREE.PlaneGeometry(20,20),new THREE.MeshStandardMaterial({color:0x182a38,roughness:.95}));floor.receiveShadow=true;floor.position.z=-.0001;scene.add(floor);
  const grid=new THREE.GridHelper(20,200,0xffffff,0xffffff);grid.rotation.x=Math.PI/2;grid.position.z=.00015;grid.material.transparent=true;grid.material.opacity=.7;scene.add(grid);
  const geometries=new Map(),meshes=[];
  for(let i=0;i<model.ngeom;i++) {
    if(model.geom_group[i]===3||model.geom_rgba[4*i+3]<.01||model.geom_type[i]===0)continue;
    let geometry,name="";
    if(model.geom_type[i]===7) {
      const id=model.geom_dataid[i];
      const accessor=model.mesh(id);name=accessor.name;accessor.delete();
      if(!geometries.has(id)) {
        geometry=new THREE.BufferGeometry();
        const va=model.mesh_vertadr[id],vn=model.mesh_vertnum[id],fa=model.mesh_faceadr[id],fn=model.mesh_facenum[id];
        geometry.setAttribute('position',new THREE.BufferAttribute(Float32Array.from(model.mesh_vert.slice(3*va,3*(va+vn))),3));
        geometry.setIndex(new THREE.BufferAttribute(Uint32Array.from(model.mesh_face.slice(3*fa,3*(fa+fn))),1));
        geometry.computeVertexNormals();geometries.set(id,geometry);
      }
      geometry=geometries.get(id);
    } else if(model.geom_type[i]===6)geometry=new THREE.BoxGeometry(...Array.from(model.geom_size.slice(i*3,i*3+3),v=>2*v));
    else continue;
    const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:new THREE.Color(robotColour(name)),roughness:.65,metalness:.05,flatShading:true}));
    mesh.castShadow=true;mesh.receiveShadow=true;mesh.matrixAutoUpdate=false;scene.add(mesh);meshes.push({mesh,id:i,name});
  }
  const previous=new THREE.Vector3(data.qpos[0],data.qpos[1],0);
  new ResizeObserver(()=>{
    const {width,height}=container.getBoundingClientRect();renderer.setSize(width,height);camera.aspect=width/height;camera.updateProjectionMatrix();
  }).observe(container);
  return {count:meshes.length,setPalette(palette){
    for(const {mesh,name} of meshes)mesh.material.color.set(robotColour(name,palette));
  },draw(follow=true){
    for(const {mesh,id:i} of meshes) {
      const r=data.geom_xmat,p=data.geom_xpos,ri=i*9,pi=i*3;
      mesh.matrix.set(r[ri],r[ri+1],r[ri+2],p[pi],r[ri+3],r[ri+4],r[ri+5],p[pi+1],r[ri+6],r[ri+7],r[ri+8],p[pi+2],0,0,0,1);
      mesh.matrixWorldNeedsUpdate=true;
    }
    const now=new THREE.Vector3(data.qpos[0],data.qpos[1],0);
    if(follow){const delta=now.clone().sub(previous);controls.target.add(delta);camera.position.add(delta);}
    previous.copy(now);light.position.set(now.x+.4,now.y-.5,1);light.target.position.copy(now);light.target.updateMatrixWorld();
    controls.update();renderer.render(scene,camera);
  }};
}
