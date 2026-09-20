import loadMujoco from './vendor/mujoco/mujoco.js';
import {Simulation} from './simulation.js';
import {createScene} from './scene.js';
const $=id=>document.getElementById(id);
let sim,view,running=false,last=0,accumulator=0,lastMetric=0;
async function get(path,type='json') {
  const response=await fetch(new URL(path,import.meta.url));
  if(!response.ok)throw Error(`Loading failed (${response.status}): ${path}`);
  return response[type]();
}
function updateMetrics() {
  const m=sim.metrics;
  $('time').innerHTML=`${m.time.toFixed(1)} <small>s</small>`;
  $('distance').innerHTML=`${(100*m.distance).toFixed(1)} <small>cm</small>`;
  $('velocity').innerHTML=`${(100*m.speed).toFixed(1)} <small>cm/s</small>`;
  $('contact').textContent=m.contacts.every(Boolean)?'Both':m.contacts[0]?'Left':m.contacts[1]?'Right':'Airborne';
  $('roll').textContent=(m.roll*180/Math.PI).toFixed(1)+'°';$('pitch').textContent=(m.pitch*180/Math.PI).toFixed(1)+'°';
}
function playback(value) {
  running=value;accumulator=0;last=0;
  $('state').textContent=sim.fallen?'Fallen · reset to centre':value?(sim.settling?'Returning to neutral':sim.turn<0?'Turning left':sim.turn>0?'Turning right':sim.drive<0?'Backward':'Forward'):'Paused';
  for(const button of document.querySelectorAll('[data-drive]')){
    button.disabled=sim.fallen;
    const active=value&&Number(button.dataset.drive)===sim.drive&&Number(button.dataset.turn)===sim.turn;
    button.classList.toggle('active',active);button.setAttribute('aria-pressed',String(active));
  }
}
function frame(now) {
  if(running) {
    if(last)accumulator+=Math.min((now-last)/1000,.08);
    let steps=0;
    while(accumulator>=sim.dt&&steps<40&&!sim.fallen){sim.step();accumulator-=sim.dt;steps++;}
    if(sim.fallen)playback(false);
  }
  last=now;view.draw($('follow').checked);
  if(now-lastMetric>120){updateMetrics();lastMetric=now;}
  requestAnimationFrame(frame);
}
export async function boot() {
  const [mj,files,gaits,xml]=await Promise.all([loadMujoco(),get('./assets/files.json'),get('./assets/gaits.json'),get('./assets/robot.xml','text')]);
  mj.FS.mkdir('/robot');mj.FS.mkdir('/robot/meshes');
  $('load-detail').textContent='Preparing CAD geometry…';
  await Promise.all(files.map(async file=>mj.FS.writeFile('/robot/'+file,new Uint8Array(await get('./assets/'+file,'arrayBuffer')))));
  mj.FS.writeFile('/robot/robot.xml',xml);
  const model=mj.MjModel.from_xml_path('/robot/robot.xml');
  sim=new Simulation(mj,model,gaits);view=createScene($('viewport'),model,sim.data);
  let palette='bordeaux';
  const paletteButton=$('palette');
  paletteButton.disabled=false;
  paletteButton.addEventListener('click',()=>{
    palette=palette==='bordeaux'?'arctic':'bordeaux';
    view.setPalette(palette);
    paletteButton.dataset.palette=palette;
    paletteButton.setAttribute('aria-pressed',String(palette==='arctic'));
    $('palette-label').textContent=palette==='bordeaux'?'Bordeaux':'White / Orange / Cyan';
  });
  $('controls').disabled=false;$('loading').hidden=true;
  const keys=new Set();
  const steer=(drive,turn)=>{if(sim.fallen)return;sim.settling=false;sim.drive=drive;sim.turn=turn;playback(true);};
  for(const button of document.querySelectorAll('[data-drive]'))button.addEventListener('click',()=>steer(Number(button.dataset.drive),Number(button.dataset.turn)));
  $('pause').addEventListener('click',()=>{keys.clear();playback(false);});
  $('reset').addEventListener('click',()=>{keys.clear();sim.drive=1;sim.turn=0;sim.reset();playback(false);updateMetrics();});
  const bindings={KeyW:'up',ArrowUp:'up',KeyS:'down',ArrowDown:'down',KeyA:'left',ArrowLeft:'left',KeyD:'right',ArrowRight:'right'};
  const keyboard=()=>{
    if(!keys.size){
      // Keep advancing physics while the gait controller returns to phase zero.
      // This is a neutral stop, distinct from pausing the simulation clock.
      if(!sim.fallen){sim.settling=true;sim.drive=0;sim.turn=0;sim.phaseTime=0;playback(true);}
      return;
    }
    const pressed=new Set(Array.from(keys,code=>bindings[code]));
    steer(pressed.has('down')?-1:1,(pressed.has('right')?1:0)-(pressed.has('left')?1:0));
  };
  document.addEventListener('keydown',event=>{
    if(event.code==='Space'&&!['BUTTON','INPUT'].includes(event.target.tagName)){event.preventDefault();keys.clear();playback(false);return;}
    if(!bindings[event.code]||['INPUT','SELECT','TEXTAREA'].includes(event.target.tagName))return;
    event.preventDefault();if(keys.has(event.code))return;keys.add(event.code);keyboard();
  });
  document.addEventListener('keyup',event=>{if(keys.delete(event.code)){event.preventDefault();keyboard();}});
  const pauseHidden=()=>{keys.clear();playback(false);};
  window.addEventListener('blur',pauseHidden);
  document.addEventListener('visibilitychange',()=>{if(document.hidden)pauseHidden();});
  playback(false);updateMetrics();requestAnimationFrame(frame);
}
