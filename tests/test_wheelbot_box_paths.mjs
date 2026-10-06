import assert from 'node:assert/strict';
import fs from 'node:fs';
const p=JSON.parse(fs.readFileSync('assets/wheelbot/live_profile.json'));
assert.equal(p.modelMetadata.baseGeometryType,'box');
assert.deepEqual(p.modelMetadata.baseInertiaKgM2,[(.07**2+.075**2)/3,(.085**2+.075**2)/3,(.085**2+.07**2)/3]);
const app=fs.readFileSync('src/wheelbot_app.mjs','utf8');
assert(!app.includes('contact_model.xml')&&!app.includes('jump_profile.json')&&!app.includes('action_jump.json'));
console.log('One-box public asset contract passed');
