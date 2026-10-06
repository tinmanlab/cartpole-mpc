"""Compile low-pose stationary charged jumps for the keyboard game.

Three independently verified schedules are generated from the existing
low-pose target-jump synthesis. Each uses the same canonical plant, no root
actuation, no reset, and no external jump force. Scheduled TVLQR/KF gains are
stored only through 1.4 s; terminal recovery returns to the original x.
"""
from pathlib import Path
import hashlib, json, time
import numpy as np
import mujoco
from scipy.linalg import solve
import design_wheelbot_target_jump as tj

ROOT=tj.ROOT
m=tj.m
ix=tj.ix
limits=tj.limits
base=tj.base
atlas=tj.atlas
p=min(atlas['profiles'],key=lambda q:abs(q['qref'][1]-.3925)+abs(q['qref'][2]))
SCHEDULED=140
CASES=[
 ('low',(.18,.30,.32,.4,-.06,.02)),
 ('medium',(.10,.24,.40,.4,-.06,.02)),
 ('high',(.18,.28,.32,.4,-.10,.02)),
]

def compile_profile(name,c):
 xs,us=tj.trace(p,c)
 r=tj.metrics(xs,us)
 x_range=[float(xs[:,0].min()),float(xs[:,0].max())]
 metrics={
  'passed':bool(r['passed']),
  'wheelClearanceM':float(r['apex'][1]-.05),
  'flightS':float(r['flightS']),
  'xRangeM':x_range,
  'maxHorizontalExcursionM':float(max(abs(x_range[0]),abs(x_range[1]))),
  'landingTangentialSpeed':float(r['landingTangentialSpeed']),
  'airborneWheelRate':float(r['airborneWheelRate']),
  'penetrationM':float(r['penetration']),
  'jointOverrunRad':float(r['jointOverrun']),
  'pitchRad':float(r['pitch']),
  'terminalRate':float(r['terminalRate']),
 }
 if not metrics['passed'] or metrics['maxHorizontalExcursionM']>.06 or metrics['airborneWheelRate']>=25:
  raise ValueError(name+': '+json.dumps(metrics))
 if len(us)!=600: raise ValueError(name+': incomplete exact-state trace')

 As=[];Bs=[]
 for k in range(SCHEDULED):
  A=np.zeros((11,11));B=np.zeros((11,3))
  for kind,size,M in [('x',11,A),('u',3,B)]:
   for j in range(size):
    pair=[]
    for eps in [1e-7,1e-8]:
     dx=np.zeros(12);du=np.zeros(3)
     if kind=='x':dx[ix[j]]=eps
     else:du[j]=eps
     xp,cp=tj.old.step(xs[k]+dx,us[k]+du);xm,cm=tj.old.step(xs[k]-dx,us[k]-du)
     pair.append(((xp-xm)[ix]/(2*eps),cp==cm))
    selected=pair[0] if pair[0][1] else pair[1]
    if not selected[1] or not np.isfinite(selected[0]).all():
     raise ValueError(f'{name}: nonlocal derivative {k}/{kind}/{j}')
    M[:,j]=selected[0]
  As.append(A);Bs.append(B)

 Q=np.array(base['Q']);R=np.array(base['R']);P=np.array(p['P']);Ks=[None]*SCHEDULED
 for k in reversed(range(SCHEDULED)):
  A=As[k];B=Bs[k]
  K=solve(R+B.T@P@B,B.T@P@A,assume_a='pos')
  F=A-B@K
  P=Q+K.T@R@K+F.T@P@F
  P=(P+P.T)/2
  Ks[k]=K

 H=np.eye(11)[[0,1,2,3,4,10]]
 Qe=np.array(base['Qe']);Re=np.array(base['Re'])
 L0=np.array(base['L']);Pe=np.array(base['kalmanPredictedCovariance']);I=np.eye(11)
 Pe=(I-L0@H)@Pe@(I-L0@H).T+L0@Re@L0.T
 Ls=[]
 for A in As:
  pred=A@Pe@A.T+Qe
  L=solve(H@pred@H.T+Re,H@pred,assume_a='pos').T
  Pe=(I-L@H)@pred@(I-L@H).T+L@Re@L.T
  Pe=(Pe+Pe.T)/2
  Ls.append(L)

 return {
  'id':name,
  'parameters':list(c),
  'entry':xs[0].tolist(),
  'ref':xs[:SCHEDULED+1].tolist(),
  'u':us[:SCHEDULED].tolist(),
  'A':np.asarray(As).tolist(),
  'B':np.asarray(Bs).tolist(),
  'K':np.asarray(Ks).tolist(),
  'L':np.asarray(Ls).tolist(),
  'referenceMetrics':metrics,
 }

def main():
 start=time.perf_counter()
 profiles=[compile_profile(name,c) for name,c in CASES]
 heights=[q['referenceMetrics']['wheelClearanceM'] for q in profiles]
 if heights!=sorted(heights): raise ValueError('stationary jump clearance is not monotone')
 terminal={key:p[key] for key in ['qref','uref','A','B','K','L']}
 terminal['qref']=list(terminal['qref']);terminal['qref'][0]=0.0
 bundle={
  'schema':'wheelbot-stationary-jump/v2',
  'assetSha256':hashlib.sha256((ROOT/'assets/wheelbot/live_model.xml').read_bytes()).hexdigest(),
  'baselineSha256':hashlib.sha256((ROOT/'assets/wheelbot/live_profile.json').read_bytes()).hexdigest(),
  'nativeVersion':mujoco.mj_versionString(),
  'controlDt':.01,'physicsDt':.002,'steps':600,'scheduledSteps':SCHEDULED,
  'controlledIndices':ix,'measurementIndices':base['measurementIndices'],
  'measurementSigma':base['measurementSigma'],'limitsNm':limits.tolist(),
  'terminal':terminal,'profiles':profiles,
  'entryTolerance':{'height':.015,'pitch':.04,'joints':.06,'rates':.3},
  'claim':'Low-pose stationary charged jumps; same plant, no root actuation/reset/external jump force. Terminal recovery returns to launch x.',
  'offlineSeconds':time.perf_counter()-start,
 }
 (ROOT/'assets/wheelbot/stationary_jump.json').write_text(json.dumps(bundle,separators=(',',':'),allow_nan=False)+'\n')
 print(json.dumps({'profiles':[{'id':q['id'],**q['referenceMetrics']} for q in profiles],'offlineSeconds':bundle['offlineSeconds']},indent=2))

if __name__=='__main__':main()
