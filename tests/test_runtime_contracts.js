'use strict';
const assert=require('assert'),fs=require('fs'),Lab=require('../src/engine');
const spec=new Lab.LabPlant().spec;
const c=new Lab.LinearMPCController(spec);c.act([.3,.1,.08,-.05],0);
assert.strictEqual(c.lastSolver,'quadprog-goldfarb-idnani','live linear MPC must call the pinned upstream QP solver');
assert(c.lastConverged && c.lastKktResidual<1e-7,'solver acceptance needs a residual, not merely lower cost');
assert(fs.existsSync('assets/cartpole.xml'),'one canonical MJCF asset is required');
assert(fs.existsSync('src/mujoco_backend.mjs'),'actual WASM physics adapter is required');
assert(fs.existsSync('vendor/mujoco/mujoco.wasm'),'the published site must contain its pinned WASM, not depend on node_modules/CDN');
assert.equal(typeof Lab.setPhysicsBackend,'function');
const qp=require('../src/qp');
const bounded=qp.solve(c.A,c.B,c.Q,c.R,c.Qf,[.3,.1,.08,-.05],c.N,c.limit,{positionLimit:.35,goal:0});
const bx=Lab.linearRollout(c.A,c.B,[.3,.1,.08,-.05],bounded.U);
assert(Math.max(...bx.map(x=>Math.abs(x[0])))<=.35+1e-8,'explicit rail constraints cannot be replaced by a cost penalty');
assert.throws(()=>qp.solve(c.A,c.B,c.Q,c.R,c.Qf,[.4,0,0,0],c.N,c.limit,{positionLimit:.35,goal:0}),'initially infeasible rail state must be rejected');
const hard=Lab.makeController('hard_mpc',spec);hard.act([.1,0,.02,0],.5);
assert(hard.lastConverged && hard.positionLimit===2.4);
console.log('runtime and hard state-constraint contracts: PASS');

// Public teaching must describe the numerical implementation actually selected above.
const vm=require('vm');
const topic=vm.runInNewContext(fs.readFileSync('src/topics.js','utf8')+';CONTROL_LAB_TOPICS.linear_mpc');
assert(topic.body.includes('quadprog'),'Linear MPC lesson must name the upstream solver');
assert(!topic.body.includes('projected gradient'),'Obsolete Linear MPC solver explanation');
