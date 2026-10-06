import assert from 'node:assert/strict';
import {planAffineTracking} from '../src/wheelbot_affine_tracking.mjs';
// A one-dimensional integrator must anticipate a changing target, rather than
// treating every point as an unrelated static equilibrium.
const I=[[1]],B=[[1]],Q=[[1]],R=[[1]];
const plan=planAffineTracking([
 {A:I,B,Q,R,trim:[0],uref:[0],reference:[0]},
 {A:I,B,Q,R,trim:[0],uref:[0],reference:[1]},
],[[1]],[1]);
assert.equal(plan.length,2);assert(Math.abs(plan[0].feedforward[0]-.6)<1e-10);
assert(Math.abs(plan[1].feedforward[0])<1e-10);
// Exact two-stage optimum: u0=.6,u1=.2,x1=.6,x2=.8.
const u0=plan[0].feedforward[0],u1=plan[1].feedforward[0]-plan[1].K[0][0]*(u0-1);
assert(Math.abs(u1-.2)<1e-10);
assert.throws(()=>planAffineTracking([{A:I,B,Q,R,trim:[NaN],uref:[0],reference:[0]}],I,[0]));
console.log('Finite-horizon affine reference tracking algebra PASS');
