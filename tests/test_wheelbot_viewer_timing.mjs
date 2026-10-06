import fs from 'node:fs';
import assert from 'node:assert/strict';
// Execute the actual public pause body with only its lexical timing dependencies.
const text=fs.readFileSync('src/wheelbot_app.mjs','utf8');
const body=text.match(/pause\(\)\{([^}]+)\}/)?.[1];assert(body,'Public pause method missing');
let calls=0;const meter={pause(){calls++;}},pureMeter={pause(){calls++;}};
const paused=new Function('meter','pureMeter',`let playing=true,previous=1,accumulator=.009;${body};return !playing`)(meter,pureMeter);
assert(paused);assert.equal(calls,2,'Paused viewer must pause both timing anchors');
assert(text.includes('playing=false;meter.pause();pureMeter.pause();'),'Manual stepping must not be reported as live wall-clock playback');
console.log('Public pause/manual stepping timing boundaries PASS');
