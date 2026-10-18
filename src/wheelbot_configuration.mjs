// One narrow XML transformation shared by browser and native-test CLI.
// Original wheel-only benchmark remains byte-identical and separately named.
const clone=x=>JSON.parse(JSON.stringify(x));
const keys=(object,expected,name)=>{if(!object||Array.isArray(object)||Object.keys(object).sort().join('|')!==expected.slice().sort().join('|'))throw Error('Unknown/missing '+name+' fields');};
const range=(x,lo,hi,name)=>{if(!Number.isFinite(x)||x<lo||x>hi)throw Error(name+' outside editor range ['+lo+','+hi+']');};
const vector=(a,n,lo,hi,name)=>{if(!Array.isArray(a)||a.length!==n)throw Error('Invalid '+name);a.forEach(x=>range(x,lo,hi,name));};
export function validateWheelbotConfiguration(c){
 keys(c,['schema','base','upper','lower','wheel','motor','contact','obstacle'],'configuration');
 if(c.schema!=='wheelbot-contact-design/v1')throw Error('Unsupported configuration schema');
 keys(c.base,['massKg','sizeM'],'base');range(c.base.massKg,.05,10,'base mass [kg]');vector(c.base.sizeM,3,.03,.8,'base dimensions [m]');
 for(const name of ['upper','lower']){keys(c[name],['massKg','lengthM','radiusM'],name);range(c[name].massKg,.01,3,name+' mass');range(c[name].lengthM,.05,.8,name+' length');range(c[name].radiusM,.004,.06,name+' radius');if(c[name].radiusM*2>c[name].lengthM)throw Error('Link radius incompatible with length');}
 keys(c.wheel,['massKg','radiusM','widthM'],'wheel');range(c.wheel.massKg,.02,3,'wheel mass');range(c.wheel.radiusM,.025,.2,'wheel radius');range(c.wheel.widthM,.01,.2,'wheel width');
 keys(c.motor,['torqueLimitNm','dampingNms','armatureKgM2','hipRangeRad','kneeRangeRad'],'motor');vector(c.motor.torqueLimitNm,3,.05,30,'torque limits [Nm]');range(c.motor.dampingNms,0,2,'joint viscous damping');range(c.motor.armatureKgM2,0,.1,'reflected rotor inertia');
 for(const joint of ['hipRangeRad','kneeRangeRad']){vector(c.motor[joint],2,-3.1,3.1,joint);if(c.motor[joint][0]>=0||c.motor[joint][1]<=0)throw Error('Joint range must bracket zero');}
 if(c.motor.hipRangeRad[1]<.15||c.motor.kneeRangeRad[0]>-.3)throw Error('Joint ranges must include the declared display/design posture hip0.15,knee-0.3rad');
 keys(c.contact,['enabled','friction','timeConstantS','dampingRatio'],'contact');if(c.contact.enabled!==true)throw Error('Full collision geometry cannot be disabled in the contact design');range(c.contact.friction,.05,2,'friction');range(c.contact.timeConstantS,.006,.05,'contact time constant');range(c.contact.dampingRatio,.5,2,'contact damping ratio');
 keys(c.obstacle,['enabled','centerM','sizeM'],'obstacle');if(typeof c.obstacle.enabled!=='boolean')throw Error('Invalid obstacle toggle');vector(c.obstacle.centerM,3,-3,3,'obstacle position');vector(c.obstacle.sizeM,3,.01,1,'obstacle dimensions');if(Math.abs(c.obstacle.centerM[1])>1e-12||c.obstacle.centerM[2]<c.obstacle.sizeM[2]/2-1e-12)throw Error('Obstacle must intersect sagittal plane and remain above floor');
 return true;
}
const fmt=v=>Array.isArray(v)?v.map(fmt).join(' '):String(Object.is(v,-0)?0:v);
function updateTag(xml,tag,name,attributes){
 const re=new RegExp('<'+tag+'\\b[^>]*\\bname="'+name+'"[^>]*>','g');const matches=xml.match(re);
 if(!matches||matches.length!==1)throw Error('Template missing/ambiguous unique '+tag+' '+name);
 let text=matches[0];for(const [key,value]of Object.entries(attributes)){
  const attr=new RegExp('\\s'+key+'="[^"]*"');const output=' '+key+'="'+fmt(value)+'"';text=attr.test(text)?text.replace(attr,output):text.replace(/\s*\/?>(?=$)/,suffix=>output+suffix);
 }
 return xml.replace(re,text);
}
function inertiaForBox(m,size){const [x,y,z]=size;return[m*(y*y+z*z)/12,m*(x*x+z*z)/12,m*(x*x+y*y)/12];}
function inertiaForRod(m,r,L){return[m*(3*r*r+L*L)/12,m*(3*r*r+L*L)/12,m*r*r/2];}
function updateBodyInertia(xml,name,position,mass,inertia){
 const re=new RegExp('(<body\\b[^>]*name="'+name+'"[^>]*>[\\s\\S]*?<inertial\\b)[^>]*(/>)');
 if(!re.test(xml))throw Error('Template missing inertial '+name);
 return xml.replace(re,'$1 pos="'+fmt(position)+'" mass="'+fmt(mass)+'" diaginertia="'+fmt(inertia)+'" $2');
}
export function buildConfiguredWheelbotXml(template,configuration){
 validateWheelbotConfiguration(configuration);const c=clone(configuration);let xml=template;
 if(typeof template!=='string'||!template.startsWith('<mujoco ')||template.includes('<include')||template.includes('<plugin'))throw Error('Unsupported model template');
 xml=xml.replace('model="planar-wheelbot-upkie-derived"','model="planar-wheelbot-full-contact-configured"');
 const baseI=inertiaForBox(c.base.massKg,c.base.sizeM);xml=updateBodyInertia(xml,'torso',[0,0,c.base.sizeM[2]/2],c.base.massKg,baseI);
 xml=updateTag(xml,'geom','torso_visual',{pos:[0,0,c.base.sizeM[2]/2],size:c.base.sizeM.map(x=>x/2),contype:1,conaffinity:1});
 for(const [configName,bodyName,geomName]of [['upper','upper_link','upper_link_visual'],['lower','lower_link','lower_link_visual']]){
  const link=c[configName];xml=updateBodyInertia(xml,bodyName,[0,0,-link.lengthM/2],link.massKg,inertiaForRod(link.massKg,link.radiusM,link.lengthM));
  xml=updateTag(xml,'geom',geomName,{pos:[0,0,-link.lengthM/2],size:[link.radiusM,link.lengthM/2],contype:1,conaffinity:1});
 }
 xml=updateTag(xml,'body','lower_link',{pos:[0,0,-c.upper.lengthM]});xml=updateTag(xml,'body','wheel_body',{pos:[0,0,-c.lower.lengthM]});
 const w=c.wheel;const transverse=w.massKg*(3*w.radiusM*w.radiusM+w.widthM*w.widthM)/12;
 xml=updateBodyInertia(xml,'wheel_body',[0,0,0],w.massKg,[transverse,w.massKg*w.radiusM*w.radiusM/2,transverse]);
 xml=updateTag(xml,'geom','wheel_visual',{size:[w.radiusM,w.widthM/2],contype:1,conaffinity:1});
 for(const [i,name]of ['hip','knee','wheel'].entries()){
  xml=updateTag(xml,'joint',name,{damping:c.motor.dampingNms,armature:c.motor.armatureKgM2,...(name==='wheel'?{}:{range:c.motor[name+'RangeRad']})});
  xml=updateTag(xml,'motor',name+'_motor',{ctrlrange:[-c.motor.torqueLimitNm[i],c.motor.torqueLimitNm[i]],forcelimited:'true',forcerange:[-c.motor.torqueLimitNm[i],c.motor.torqueLimitNm[i]]});
 }
 if(!xml.includes('<geom condim="3" friction="1 0.01 0.001" solref="0.01 1"'))throw Error('Unsupported contact defaults in template');
 xml=xml.replace('<geom condim="3" friction="1 0.01 0.001" solref="0.01 1"','<geom condim="3" friction="'+fmt(c.contact.friction)+' 0.01 0.001" solref="'+fmt(c.contact.timeConstantS)+' '+fmt(c.contact.dampingRatio)+'"');
 if(c.obstacle.enabled)xml=xml.replace('    <body name="torso"','    <geom name="obstacle" type="box" pos="'+fmt(c.obstacle.centerM)+'" size="'+fmt(c.obstacle.sizeM.map(x=>x/2))+'" rgba="0.65 0.45 0.23 1" contype="1" conaffinity="1"/>\n    <body name="torso"');
 const hip=.15,knee=-.3;const z=w.radiusM+c.upper.lengthM*Math.cos(hip)+c.lower.lengthM*Math.cos(hip+knee);
 xml=updateTag(xml,'key','home',{qpos:[0,z,0,hip,knee,0]});
 const metadata={schema:'wheelbot-configured-metadata/v1',totalMassKg:c.base.massKg+c.upper.massKg+c.lower.massKg+c.wheel.massKg,baseInertiaKgM2:baseI,motorTorqueLimitsNm:c.motor.torqueLimitNm,configuration:c,initialState:[0,z,0,hip,knee,0,0,0,0,0,0,0],fullBodyContact:true,collisionExclusion:'MuJoCo parent-child filter only; world contacts and nonadjacent self-contact enabled',actuatorScope:'Ideal torque ceiling, viscous joint damping and reflected rotor inertia; no thermal, torque-speed, delay or compliance model',controllerProfileValid:false};
 return {xml,metadata};
}
