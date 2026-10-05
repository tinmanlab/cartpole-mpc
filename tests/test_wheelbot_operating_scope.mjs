// Exercise the actual public preparation function without HTTP or browser mocks.
import fs from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';
const source=fs.readFileSync('src/wheelbot_app.mjs','utf8');
const html=fs.readFileSync('wheelbot.html','utf8');
const code=source.slice(source.indexOf('function prepareForce('),source.indexOf('function tick('));
let creates=0;
const fields={mode:{value:'lqr_kf'}};
const context={ready:true,forceProtocol:{controllers:['lqr_kf','mpc_kf'],stressN:40},forceOperating:{amplitudeN:.8},profiles:{recovery:{}},backend:{},playing:true,accumulated:.02,droppedWallSeconds:.1,poseOnly:false,statusLocked:false,jumpActive:true,jumpInitialTelemetry:{},trial:{steps:17},forceActive:false,viewState:[1],$:id=>fields[id]??=( {} ),createForceTrial:()=>{creates++;throw Error('trial creation reached');}};
vm.createContext(context);vm.runInContext(code,context);
const snapshot=()=>JSON.stringify({playing:context.playing,accumulated:context.accumulated,droppedWallSeconds:context.droppedWallSeconds,poseOnly:context.poseOnly,statusLocked:context.statusLocked,jumpActive:context.jumpActive,jumpInitialTelemetry:context.jumpInitialTelemetry,trial:context.trial,forceActive:context.forceActive,viewState:context.viewState,fields});
for(const level of ['stress','40',40,NaN,null,'']){
 const before=snapshot();assert.throws(()=>context.prepareForce(1,level),/normal/i);assert.equal(snapshot(),before);assert.equal(creates,0);
}
for(const direction of [NaN,0,'1']){const before=snapshot();assert.throws(()=>context.prepareForce(direction,'normal'),/direction/i);assert.equal(snapshot(),before);assert.equal(creates,0);}
assert(!/40\s*N|stress|<select id="force-level"/i.test(html),'normal page must not expose stress or a force selector');
for(const mode of ['lqr_kf','mpc_kf'])for(const direction of [-1,1]){fields.mode.value=mode;assert.throws(()=>context.prepareForce(direction,'normal'),/trial creation reached/);}
assert.equal(creates,4);
console.log('Operating scope: invalid public requests preserve state; normal signs/modes reach trial creation; UI has no stress selector PASS');

const contactLine=source.split('\n').find(line=>line.includes("$('contact').textContent"));
for(const count of [0,1]){
 const contactFields={contact:{textContent:''}};
 vm.runInNewContext(contactLine,{contact:{wheelContacts:count,slip:1.2345},s:{steps:12,failed:false},$:id=>contactFields[id]});
 assert(contactFields.contact.textContent.includes(count===0?'airborne surface velocity':'slip'));
 assert(contactFields.contact.textContent.includes('1.2345 m/s'));
}
console.log('Contact-dependent label preserves the numeric surface velocity PASS');
