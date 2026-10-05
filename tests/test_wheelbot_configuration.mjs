import assert from 'node:assert/strict';
import fs from 'node:fs';
assert(fs.existsSync('src/wheelbot_configuration.mjs'),'Missing model/collision configuration builder');
const C=await import('../src/wheelbot_configuration.mjs');
const template=fs.readFileSync('assets/wheelbot/wheelbot.xml','utf8');
const defaults=JSON.parse(fs.readFileSync('assets/wheelbot/contact_design.json'));
const compiled=C.buildConfiguredWheelbotXml(template,defaults);
assert.equal(C.validateWheelbotConfiguration(defaults),true);
assert.equal(compiled.xml,C.buildConfiguredWheelbotXml(template,structuredClone(defaults)).xml);
assert.equal(compiled.metadata.totalMassKg,1.1991500000000002);
assert.deepEqual(compiled.metadata.motorTorqueLimitsNm,[16,16,1.7]);
for(const name of ['torso_visual','upper_link_visual','lower_link_visual','wheel_visual']){
 const tag=compiled.xml.match(new RegExp('<geom[^>]*name="'+name+'"[^>]*>'))?.[0];
 assert(tag?.includes('contype="1"')&&tag.includes('conaffinity="1"'),name+' collisions not enabled');
}
assert(compiled.xml.includes('name="obstacle"'));
const changed=structuredClone(defaults);changed.base.massKg=1.3;changed.upper.lengthM=.3;changed.motor.torqueLimitNm=[5,6,1];
const next=C.buildConfiguredWheelbotXml(template,changed);
assert.notEqual(next.xml,compiled.xml);
assert.deepEqual(next.metadata.motorTorqueLimitsNm,[5,6,1]);
assert(next.xml.includes('name="lower_link" pos="0 0 -0.3"'));
assert.equal(next.metadata.totalMassKg,1.4991500000000002);
for(const corrupt of [c=>c.base.massKg=-1,c=>c.upper.lengthM=NaN,c=>c.motor.torqueLimitNm[0]=Infinity,c=>c.upper.extra=1,c=>c.motor.dampingNms=-1,c=>c.unknown=1,c=>c.contact.enabled=false]){
 const bad=structuredClone(defaults);corrupt(bad);assert.throws(()=>C.buildConfiguredWheelbotXml(template,bad));
}
assert.throws(()=>C.buildConfiguredWheelbotXml(template.replace('name="torso_visual"','name="renamed"'),defaults),/missing|unique|template/i);
console.log('Parameterized mass/shape/inertia/motor and mandatory full-contact contract PASS');
