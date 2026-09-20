export const clamp=(x,lo,hi)=>Math.max(lo,Math.min(hi,x));
export function orientation(q) {
  const [w,x,y,z]=q.slice(3,7);
  return [Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),Math.asin(clamp(2*(w*y-z*x),-1,1))];
}
export function command(p,t,roll,pitch,gyro,stride=1) {
  const phase=2*Math.PI*t/p[0], s=Math.sin(phase),c=Math.cos(phase),s2=Math.sin(2*phase),c2=Math.cos(2*phase);
  const swingH=stride*(p[3]*s+p[4]*c),evenH=stride*(p[5]*s2+p[6]*c2);
  const swingK=stride*(p[7]*s+p[8]*c),evenK=stride*(p[9]*s2+p[10]*c2);
  const balance=-p[14]*pitch-p[15]*gyro[1];
  const lean=stride*(p[11]*s+p[12]*c)-p[16]*roll-p[17]*gyro[0];
  return [lean+p[13],p[1]+swingH+evenH+balance,p[2]+swingK+evenK,
          lean-p[13],p[1]-swingH+evenH+balance,p[2]-swingK+evenK];
}
export class Simulation {
  constructor(mj,model,gaits) {
    this.mj=mj;this.model=model;this.data=new mj.MjData(model);this.gaits=gaits;
    this.gait=gaits[0];this.rate=1;this.stride=1;this.drive=1;this.turn=0;this.settling=false;this.settleBlend=0;this.turnState=0;this.turnGain=.10;
    this.dt=model.opt.timestep;
    this.controlSteps=Math.round(.02/this.dt);
    if(model.nu!==6||Math.abs(this.controlSteps*this.dt-.02)>1e-9)throw Error('The model must have 6 servomotors and 50 Hz control.');
    this.sensor={};
    for(const name of ['imu_gyro','L_touch','R_touch']) {
      const accessor=model.sensor(name);this.sensor[name]=Number(accessor.adr);accessor.delete();
    }
    this.reset();
  }
  reset() {
    this.mj.mj_resetDataKeyframe(this.model,this.data,0);
    this.control=Float64Array.from(this.data.qpos.slice(7,13));
    this.data.ctrl.set(this.control);
    for(let i=0;i<150;i++)this.mj.mj_step(this.model,this.data);
    this.mj.mj_forward(this.model,this.data);
    this.startX=this.data.qpos[0];this.startY=this.data.qpos[1];this.startTime=this.data.time;
    // Use the settled keyframe pose as the stop target, rather than the raw
    // actuator command before MuJoCo has relaxed the body onto its feet.
    this.neutralControl=Float64Array.from(this.data.qpos.slice(7,13));
    this.phaseTime=0;this.steps=0;this.fallen=false;this.settling=false;this.settleBlend=0;this.turnState=0;
  }
  select(id) {
    const gait=this.gaits.find(g=>g.id===id);
    if(!gait)throw Error('Unknown gait');
    this.gait=gait;this.reset();
  }
  step() {
    if(this.fallen)return;
    const d=this.data,m=this.model;
    if(this.steps%this.controlSteps===0) {
      const [roll,pitch]=orientation(d.qpos), adr=this.sensor.imu_gyro;
      const gyro=d.sensordata.slice(adr,adr+3);
      const turningInPlace=this.drive===0&&this.turn!==0;
      const settlingStride=this.settling?this.stride*(1-this.settleBlend):this.stride;
      const walkingTarget=command(this.gait.params,this.phaseTime,roll,pitch,gyro,turningInPlace?.55:settlingStride);
      const target=walkingTarget;
      if(this.settling){
        const balanced=d.sensordata[this.sensor.L_touch]>.03&&d.sensordata[this.sensor.R_touch]>.03;
        if(balanced)this.settleBlend=Math.min(1,this.settleBlend+.001);
        if(this.settleBlend>.9)for(let i=0;i<6;i++)target[i]=this.neutralControl[i];
      }
      if(this.settling&&this.settleBlend>.8){
        // The initial servo angles are the neutral pose; add a small IMU hold so
        // the body can settle without pitching over while it is still moving.
        const pitchHold=clamp(1.3*pitch+.15*gyro[1],-.24,.24);
        const rollHold=clamp(-1.5*roll-.12*gyro[0],-.18,.18);
        target[1]+=pitchHold;target[4]+=pitchHold;target[0]+=rollHold;target[3]+=rollHold;
      }
      this.turnState+=clamp((this.gait.steeringSign??1)*this.turn-this.turnState,-.06,.06);
      const mid=(target[1]+target[4])*.5;
      const gain=turningInPlace?1.0:this.turnGain;
      target[1]=mid+(target[1]-mid)*(1+gain*this.turnState);
      target[4]=mid+(target[4]-mid)*(1-gain*this.turnState);
      const maxStep=this.gait.slew*.02;
      for(let i=0;i<6;i++) {
        const next=clamp(target[i],m.actuator_ctrlrange[2*i],m.actuator_ctrlrange[2*i+1]);
        this.control[i]=clamp(this.control[i]+clamp(next-this.control[i],-maxStep,maxStep),m.actuator_ctrlrange[2*i],m.actuator_ctrlrange[2*i+1]);
      }
      d.ctrl.set(this.control);
    }
    this.mj.mj_step(m,d);this.steps++;
    const settlingStride=this.settling&&this.settleBlend===0?1:0;
    this.phaseTime+=this.dt*this.rate*(this.drive===0&&this.turn!==0?1:this.drive||settlingStride);
    const [roll,pitch]=orientation(d.qpos);
    this.fallen=!Array.from(d.qpos).every(Number.isFinite)||d.qpos[2]<.05||Math.abs(roll)>1.1||Math.abs(pitch)>1.1;
  }
  get metrics() {
    const d=this.data,[roll,pitch]=orientation(d.qpos);
    return {time:d.time-this.startTime,distance:Math.hypot(d.qpos[0]-this.startX,d.qpos[1]-this.startY),
      speed:Math.hypot(d.qvel[0],d.qvel[1]),roll,pitch,
      contacts:[d.sensordata[this.sensor.L_touch]>.03,d.sensordata[this.sensor.R_touch]>.03]};
  }
  dispose(){this.data.delete();this.model.delete();}
}
