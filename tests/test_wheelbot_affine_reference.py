"""Independent SciPy reconstruction of the browser affine LQ recursion.
No browser implementation is imported into the reference arithmetic.
"""
from pathlib import Path
import hashlib,json,subprocess
import numpy as np
from scipy.linalg import solve
ROOT=Path(__file__).resolve().parents[1]
p=json.loads((ROOT/'assets/wheelbot/live_profile.json').read_text())
A=np.array(p['A']);B=np.array(p['B']);Q=np.array(p['Q']);R=np.array(p['R']);P=np.array(p['P'])
trim=np.r_[p['qref'][:5],np.zeros(6)];stages=[]
for k in range(60):
 t=k/60;s=10*t**3-15*t**4+6*t**5;v=.1/.6*(30*t*t-60*t**3+30*t**4)
 reference=trim.copy();reference[0]=.1*s;reference[5]=v;reference[10]=v/.05
 origin=trim.copy();origin[0]=reference[0]
 stages.append({'A':A.tolist(),'B':B.tolist(),'Q':Q.tolist(),'R':R.tolist(),'trim':origin.tolist(),'uref':p['uref'],'reference':reference.tolist()})
terminal=trim.copy();terminal[0]=.1
payload={'stages':stages,'terminalP':P.tolist(),'terminalReference':terminal.tolist()}
js="import fs from 'node:fs';import{planAffineTracking}from'./src/wheelbot_affine_tracking.mjs';const x=JSON.parse(fs.readFileSync(0,'utf8'));console.log(JSON.stringify(planAffineTracking(x.stages,x.terminalP,x.terminalReference)));"
actual=json.loads(subprocess.check_output(['node','--input-type=module','-e',js],input=json.dumps(payload),text=True,cwd=ROOT))
linear=np.zeros(11);maxK=0.;maxU=0.;maxResidual=0.
for k in reversed(range(len(stages))):
 stage=stages[k];reference=np.array(stage['reference']);origin=np.array(stage['trim']);next_reference=np.array(stages[k+1]['reference'])if k+1<len(stages)else terminal
 c=A@(reference-origin)+origin-next_reference;H=R+B.T@P@B
 K=solve(H,B.T@P@A,assume_a='pos');v=P@c+linear;ff=-solve(H,B.T@v,assume_a='pos')
 maxK=max(maxK,float(np.max(abs(K-np.asarray(actual[k]['K'])))))
 maxU=max(maxU,float(np.max(abs(ff-np.asarray(actual[k]['feedforward'])))))
 maxResidual=max(maxResidual,float(np.max(abs(H@np.asarray(actual[k]['feedforward'])+B.T@v))))
 F=A-B@K;linear=F.T@v;P=Q+K.T@R@K+F.T@P@F;P=(P+P.T)/2
 assert np.linalg.eigvalsh(P).min()>0
assert maxK<1e-7 and maxU<1e-8 and maxResidual<1e-7,(maxK,maxU,maxResidual)
result={'passed':True,'stages':len(stages),'maxGainDifference':maxK,'maxFeedforwardDifferenceNm':maxU,'maxStationarityResidual':maxResidual,'modelSha256':p['assetSha256'],'method':'Independent SciPy linear solves and completed-square recursion versus pinned-Quadprog browser implementation','scope':'Algebraic consistency is separate from nonlinear contact and closed-loop acceptance.'}
(ROOT/'test-results').mkdir(exist_ok=True);(ROOT/'test-results/wheelbot-affine-reference.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
