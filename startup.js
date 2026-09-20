const $=id=>document.getElementById(id);
const reloadKey='duckbert-isolation-reload';
async function withTimeout(work,milliseconds,message) {
  let timer;
  try {
    return await Promise.race([work,new Promise((_,reject)=>{
      timer=setTimeout(()=>reject(new Error(message)),milliseconds);
    })]);
  } finally {clearTimeout(timer);}
}
async function start() {
  if(!globalThis.crossOriginIsolated) {
    if(!window.isSecureContext||!('serviceWorker' in navigator))
      throw new Error('This browser cannot enable the physics engine. Open this page directly in a recent browser over HTTPS.');
    if(sessionStorage.getItem(reloadKey))
      throw new Error('The browser could not enable the physics engine. Allow service workers and open the site directly in a new tab.');
    $('load-detail').textContent='Enabling the physics engine…';
    await withTimeout((async()=>{
      await navigator.serviceWorker.register(new URL('./isolation-worker.js',import.meta.url),{scope:'./',updateViaCache:'none'});
      await navigator.serviceWorker.ready;
      if(!navigator.serviceWorker.controller)await new Promise(resolve=>{
        navigator.serviceWorker.addEventListener('controllerchange',resolve,{once:true});
      });
    })(),30000,'Browser setup timed out. Reload the page to try again.');
    sessionStorage.setItem(reloadKey,'1');
    location.reload();
    return;
  }
  sessionStorage.removeItem(reloadKey);
  $('load-detail').textContent='Loading the physics engine…';
  await withTimeout((async()=>{
    const {boot}=await import('./app.js');
    await boot();
  })(),90000,'Loading took too long. Check your connection and reload the page.');
}
start().catch(error=>{
  $('loading').hidden=true;
  $('controls').disabled=true;
  $('error').hidden=false;
  $('error').textContent='Simulation unavailable: '+error.message;
  $('state').textContent='Loading error';
  console.error(error);
});
