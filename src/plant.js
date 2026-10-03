'use strict';
// Copyright 2025 Marc Duclusaud & Grégoire Passault (BAM source equations).
// Licensed under Apache-2.0; see licenses/Apache-2.0.txt.
// Modified: scalar JS equation port and local wheel/CartPole integration.
// Adapted from Rhoban/BAM (Apache-2.0), pinned source in vendor/bam/NOTICE.md.
// Only the voltage torque equation + M6 friction budget are ported. The wheel
// coupling, PWM command adapter, ranges and curriculum are local educational code.
const Plant=(()=>{
 const params=typeof module!=='undefined'?require('./bam_params'):BAM_PARAMS;
 const ACTUATORS={ideal:{label:'Ideal force · comparison only',vin:0,armature:0},xl330:{...params.xl330,label:'Dynamixel XL330 · BAM M6',vin:5,maxPWM:1},mx64:{...params.mx64,label:'Dynamixel MX64 · BAM M6',vin:12,maxPWM:.9625},mx106:{...params.mx106,label:'Dynamixel MX106 · BAM M6',vin:12,maxPWM:.9625}};
 const WHEEL={name:'ROBOTIS TurtleBot3 wheel',radius:.033,width:.018,mass:.028498940,inertia:2.0712558e-5,source:'turtlebot3_description/urdf/turtlebot3_burger.urdf',blob:'5b1470e40ee3431475ecd85fbd4294c36933cb40'};
 const DEFAULT_SPEC={actuator:'xl330',ratio:2,force:8,mc:1,mp:.1,l:.5,gravity:9.8,mu:1.2,modelSpread:.2,wind:1,push:6,pushMin:.1,pushMax:.24,pushPeriod:2.2,noise:1,delay:.02,lag:.02,gainSpread:.15,friction:.1,rewardPosition:.5,rewardVelocity:.02,rewardAngle:.05};
 const bounds={ratio:[1,8],force:[1,60],mc:[.1,10],mp:[.02,2],l:[.1,1.5],gravity:[1,20],mu:[.1,3],modelSpread:[0,.5],wind:[0,15],push:[0,40],pushMin:[.02,2],pushMax:[.02,2],pushPeriod:[.1,10],noise:[0,10],delay:[0,.1],lag:[0,.1],gainSpread:[0,.5],friction:[0,2],rewardPosition:[0,3],rewardVelocity:[0,1],rewardAngle:[0,5]};
 function validateSpec(raw={}){const s={...DEFAULT_SPEC,...raw};if(!Object.hasOwn(ACTUATORS,s.actuator))throw Error('Unsupported actuator');for(const [k,[lo,hi]] of Object.entries(bounds))if(!Number.isFinite(s[k])||s[k]<lo||s[k]>hi)throw Error(`${k}: ${lo}~${hi} is the supported range.`);if(s.pushMin>s.pushMax||s.pushPeriod<s.pushMax)throw Error('Invalid push duration or repeat interval.');return s;}
 const cl=(x,a,b)=>Math.min(b,Math.max(a,x));
 function motorTorque(p,voltage,speed){return p.kt*voltage/p.R-p.kt*p.kt*speed/p.R;}
 function frictionM6(p,motor,external,speed){const str=Math.exp(-(Math.abs(speed/p.dtheta_stribeck)**p.alpha));let f=p.friction_base+Math.abs(external*p.load_friction_external-motor*p.load_friction_motor);f+=str*p.friction_stribeck+str*Math.abs(external*p.load_friction_external_stribeck-motor*p.load_friction_motor_stribeck);if(Math.sign(external)!==Math.sign(motor)){if(Math.abs(external)<Math.abs(motor))f+=str*p.load_friction_external_quad*external**2;else if(Math.abs(external)>Math.abs(motor))f+=str*p.load_friction_motor_quad*motor**2;}return {frictionloss:f,damping:p.friction_viscous,stribeck:str};}
 function sample(s,profile,L,R){const u=()=>R.uniform(),both=profile==='mixed'||profile==='ood',m=both||profile==='model',a=both||profile==='actuator',ns=both||profile==='sensor',scale=profile==='ood'?1.5:L;
  const around=(base,spread)=>base*(1+(2*u()-1)*spread*scale),mc=around(s.mc,s.modelSpread),mp=around(s.mp,s.modelSpread),l=around(s.l,s.modelSpread*.5),gain=around(1,s.gainSpread),tau=u()*s.lag*scale,delay=Math.round(u()*s.delay*scale/.02),friction=u()*s.friction*scale,bias=(2*u()-1)*s.wind*scale,amp=u()*s.push*scale,dur=Math.round((s.pushMin+u()*(s.pushMax-s.pushMin))/.02),offset=60+Math.floor(u()*30),sign=u()<.5?-1:1;
  const noise=[.003,.02,.001,.02].map(x=>x*s.noise*scale*(.25+.75*u()));return {mc:m?mc:s.mc,mp:m?mp:s.mp,l:m?l:s.l,gain:a?gain:1,tau:a?tau:0,delay:a?delay:0,friction:m?friction:0,bias:both||profile==='wind'?bias:0,pushAmp:both||profile==='push'?amp:0,pushDuration:dur,pushOffset:offset,pushPeriod:Math.round(s.pushPeriod/.02),pushSign:sign,noise:ns?noise:[0,0,0,0],thetaBias:0,maxForce:s.force,actuator:s.actuator};
 }
 // 1D, no-slip constrained cart. Two driven wheel/rotor inertias reflected into
 // x. Semi-implicit 200-Hz stepping; electrical/mechanical damping implicit.
 function integrate(state,s,p,command,external,dt=.005,tipForce=0){if(!Number.isFinite(tipForce))throw Error('Invalid pole-tip force');let [x,v,t,w]=state;const a=ACTUATORS[s.actuator],n=2,r=WHEEL.radius,rg=r*s.ratio,co=Math.cos(t),sn=Math.sin(t),J=s.actuator==='ideal'?0:n*WHEEL.inertia/r**2+n*a.armature/rg**2,D=p.mc+p.mp+J-.75*p.mp*co*co;
  // Virtual work: Qx = Ftip, Qtheta = 2*l*cos(theta)*Ftip.
  // Eliminate rod acceleration using I_pivot = (4/3)*mp*l^2.
  const B=external-p.friction*v+p.mp*p.l*w*w*sn-.75*p.mp*s.gravity*sn*co+tipForce*(1-1.5*co*co);
  let voltage=0,motor=0,dry=0,damping=0,emfD=0,drive0=command,friction={frictionloss:0,damping:0,stribeck:0},saturated=false;
  if(s.actuator!=='ideal'){
   const requested=command*rg/n*a.R/a.kt,limit=a.vin*a.maxPWM;
   voltage=cl(requested,-limit,limit);saturated=Math.abs(requested)>limit+1e-12;
   motor=motorTorque(a,voltage,v/rg);friction=frictionM6(a,motor,B*rg/n,v/rg);
   drive0=n*a.kt*voltage/a.R/rg;emfD=n*a.kt*a.kt/a.R/rg**2;damping=n*friction.damping/rg**2;dry=n*friction.frictionloss/rg;
  }
  const denominator=D+dt*(damping+emfD),free=(D*v+dt*(drive0+B))/denominator,stop=Math.min(Math.abs(free),dt*dry/denominator),nextV=free-Math.sign(free)*stop;
  const resist=Math.sign(free)*stop*denominator/dt,acc=(nextV-v)/dt,alpha=(s.gravity*sn-co*acc+2*tipForce*co/p.mp)/(p.l*4/3),nextW=w+alpha*dt;
  const motorForce=s.actuator==='ideal'?command:n*motorTorque(a,voltage,nextV/rg)/rg;
  // Total normal reaction from vertical acceleration of the rod COM.
  // This remains a planar, rigid, no-slip model; a violated contact constraint
  // is a model-validity stop, not a simulation of tire slip or flight.
  const contact=motorForce-resist-damping*nextV-J*acc;
  const normalForce=(p.mc+p.mp)*s.gravity-p.mp*p.l*(alpha*sn+w*w*co);
  const tractionLimit=s.mu*Math.max(0,normalForce);
  const tractionRatio=Math.abs(contact)/Math.max(1e-12,tractionLimit);
  return {state:[x+nextV*dt,nextV,t+nextW*dt,nextW],drive:{voltage,current:s.actuator==='ideal'?0:(voltage-a.kt*nextV/rg)/a.R,motorTorque:s.actuator==='ideal'?0:motorTorque(a,voltage,nextV/rg),externalTorque:B*rg/n,rotorSpeed:nextV/rg,rotorInertia:a.armature||0,reflectedMass:J,frictionBudget:friction.frictionloss,frictionTorque:resist*rg/n,viscousTorque:s.actuator==='ideal'?0:friction.damping*nextV/rg,motorForce,contactForce:contact,normalForce,tractionLimit,contactLost:normalForce<=0,tractionRatio,saturated,acc,alpha,tipForce,tipMoment:2*p.l*co*tipForce,wheelAngle:(x+nextV*dt)/r,rotorAngle:(x+nextV*dt)/rg}};
 }
 const STAGES=['nominal','push','sensor','actuator','model','mixed'];
 function newCurriculum(plan='fixed'){if(!['fixed','ramp','staged'].includes(plan))throw Error('Invalid learning plan.');return {plan,index:0,streak:0,last:null,history:[],phaseStart:0};}
 function gradeCurriculum(c,result,iter){if(c.plan!=='staged')return false;const passed=result.passed/result.count>=.75;c.streak=passed?c.streak+1:0;c.last={iter,...result};let promoted=false;if(c.streak>=2&&c.index<STAGES.length-1){c.index++;c.streak=0;c.phaseStart=iter;promoted=true;}c.history.push({iter,passed:result.passed,count:result.count,rate:result.passed/result.count,next:STAGES[c.index],promoted});if(c.history.length>100)c.history.shift();return promoted;}
 function profileFor(h,c){return c.plan==='staged'?STAGES[c.index]:h.profile==='randomized'?'mixed':h.profile;}
 function levelFor(c,iter){return c.plan==='ramp'?Math.min(1,.25+.75*Math.max(0,iter-(c.phaseStart||0))/100):1;}
 return {ACTUATORS,WHEEL,DEFAULT_SPEC,validateSpec,motorTorque,frictionM6,sample,integrate,newCurriculum,gradeCurriculum,STAGES,profileFor,levelFor};
})();
if(typeof module!=='undefined')module.exports=Plant;
