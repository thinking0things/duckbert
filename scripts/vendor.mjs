import {mkdir,copyFile} from 'node:fs/promises';
const root=new URL('../',import.meta.url);
for(const name of ['mujoco','three'])await mkdir(new URL(`dist/vendor/${name}/`,root),{recursive:true});
for(const file of ['mujoco.js','mujoco.wasm'])await copyFile(new URL(`node_modules/@mujoco/mujoco/${file}`,root),new URL(`dist/vendor/mujoco/${file}`,root));
for(const file of ['three.module.js','three.core.js'])await copyFile(new URL(`node_modules/three/build/${file}`,root),new URL(`dist/vendor/three/${file}`,root));
await copyFile(new URL('node_modules/three/examples/jsm/controls/OrbitControls.js',root),new URL('dist/vendor/three/OrbitControls.js',root));
await copyFile(new URL('node_modules/three/LICENSE',root),new URL('dist/vendor/three/LICENSE',root));
console.log('MuJoCo and Three.js copied to dist/vendor.');
