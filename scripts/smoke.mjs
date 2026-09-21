import {readFile,writeFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import loadMujoco from '@mujoco/mujoco';
import {Simulation,command} from '../dist/simulation.js';
const root=new URL('../',import.meta.url);
const mj=await loadMujoco({wasmBinary:await readFile(new URL('node_modules/@mujoco/mujoco/mujoco.wasm',root))});
mj.FS.mkdir('/robot');mj.FS.mkdir('/robot/meshes');
for(const file of JSON.parse(await readFile(new URL('dist/assets/files.json',root),'utf8')))mj.FS.writeFile('/robot/'+file,new Uint8Array(await readFile(new URL('dist/assets/'+file,root))));
mj.FS.writeFile('/robot/robot.xml',await readFile(new URL('dist/assets/robot.xml',root),'utf8'));
const model=mj.MjModel.from_xml_path('/robot/robot.xml');
const provenance=JSON.parse(await readFile(new URL('dist/assets/provenance.json',root),'utf8'));
assert.equal(provenance.cad_revision,'out_none_v1.1');
assert.equal(createHash('sha256').update(await readFile(new URL('dist/assets/robot.xml',root))).digest('hex'),provenance.model_sha256);
for(const [name,hash] of Object.entries(provenance.mesh_sha256))
  assert.equal(createHash('sha256').update(await readFile(new URL('dist/assets/meshes/'+name,root))).digest('hex'),hash,'CAD mesh changed: '+name);
assert.equal(Object.keys(provenance.mesh_sha256).length,32);
assert(provenance.mesh_sha256['15_lipo_cassette.stl']);
assert(provenance.mesh_sha256['ESP32_C3_shield_envelope.stl']);
for(let i=0;i<model.ngeom;i++)if(model.geom_group[i]===1){
  const rgba=model.geom_rgba.slice(4*i,4*i+4);
  assert(Math.abs(rgba[0]-.2158605)<1e-6&&rgba[1]===0&&Math.abs(rgba[2]-.01444384)<1e-6&&rgba[3]===1,'Every robot visual must be Bordeaux red');
}

const gaits=JSON.parse(await readFile(new URL('dist/assets/gaits.json',root),'utf8'));
const sim=new Simulation(mj,model,gaits);
const reports=[];
assert.equal(gaits.length,1,'The site must expose exactly one gait');
for(const [name,drive,turn] of [['forward',1,0],['left',1,-1],['right',1,1],['backward',-1,0]]){
  sim.select(gaits[0].id);sim.drive=drive;sim.turn=turn;
  const start=Date.now();
  for(let i=0;i<6000&&!sim.fallen;i++)sim.step();
  const [w,x,y,z]=sim.data.qpos.slice(3,7);
  const heading_deg=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))*180/Math.PI;
  reports.push({direction:name,heading_deg,x_m:sim.data.qpos[0],...sim.metrics,fallen:sim.fallen,wall_seconds:(Date.now()-start)/1000});
  console.log(JSON.stringify(reports.at(-1)));
}
await writeFile(new URL('simulation-check.json',root),JSON.stringify(reports,null,2));
assert(reports.every(r=>!r.fallen&&r.distance>.1),'A direction did not complete the physics check');
assert(reports[1].heading_deg>reports[0].heading_deg,'Left command must increase heading');
assert(reports[2].heading_deg<reports[0].heading_deg,'Right command must decrease heading');
assert(reports[3].x_m<0,'Backward command must move backwards');
const html=await readFile(new URL('dist/index.html',root),'utf8');
const app=await readFile(new URL('dist/app.js',root),'utf8');
for(const match of app.matchAll(/\$\('([^']+)'\)/g))assert(html.includes(`id="${match[1]}"`),`Missing control ${match[1]}`);
assert(!html.includes('id="gait"'),'No gait chooser');
assert(!app.includes('serial')&&!app.includes('WebSocket'),'No hardware connection');
sim.dispose();
