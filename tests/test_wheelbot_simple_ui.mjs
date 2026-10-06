import fs from 'node:fs';
import assert from 'node:assert/strict';
const html=fs.readFileSync('wheelbot.html','utf8');
const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);
assert.equal(new Set(ids).size,ids.length,'No duplicate control IDs');
for(const id of ['target-x','target-z','target-pitch','action-jump','action-recover','disturb-kind','disturb-strength','disturb-left','disturb-right','advanced-lab'])assert(ids.includes(id),'Missing direct control '+id);
assert(html.indexOf('id="view"')<html.indexOf('id="advanced-lab"'),'Viewport before advanced experiments');
assert(!/<details[^>]+id="advanced-lab"[^>]*\bopen\b/.test(html),'Advanced controls closed by default');
assert(html.includes('목표')&&html.includes('점프')&&html.includes('일어서기'),'Plain user-facing action names');
console.log('Direct pose/action controls and collapsed advanced workspace PASS');

const app=fs.readFileSync('src/wheelbot_app.mjs','utf8');assert(app.includes("if($('target-x').disabled)return;"),'Keyboard must respect disabled pose controls');
