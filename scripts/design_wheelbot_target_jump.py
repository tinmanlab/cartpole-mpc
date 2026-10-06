"""Bounded box jump family. Exact-state offline synthesis; scheduled KF runtime."""
from pathlib import Path
import json,time,hashlib,itertools
import numpy as np
import mujoco
from scipy.optimize import least_squares
import design_wheelbot_action_jump as old
ROOT=old.ROOT;m=old.m;ix=old.idx;limits=old.limits
PROTOCOL=dict(physicsDt=.002,controlDt=.01,minFlightS=.03,minWheelClearanceM=.02,maxTargetErrorM=.03,maxPenetrationM=.01,maxJointOverrunRad=.02,maxPitchRad=.6,maxTerminalRate=.3,maxAirborneWheelRate=25.,targetMeaning='wheel-centre apex world x,z; landing on existing floor',durationS=6.,seeds=[7,42],maxLandingTangentialSpeed=.5,landingSpeedScope='Explicit simulation benchmark assumption (0.5 m/s), not a hardware-derived bound')
atlas=json.loads((ROOT/'assets/wheelbot/pose_profiles.json').read_text());base=old.p
scratch=mujoco.MjData(m);d=mujoco.MjData(m);wheel=m.geom('wheel_visual').id

def smooth(t,points):
 for (a,x),(b,y) in zip(points,points[1:]):
  if t<b:
   f=np.clip((t-a)/(b-a),0,1);s=10*f**3-15*f**4+6*f**5;ds=(30*f*f-60*f**3+30*f**4)/(b-a);dds=(60*f-180*f*f+120*f**3)/(b-a)**2
   return x+(y-x)*s,(y-x)*ds,(y-x)*dds
 return points[-1][1],0.,0.

def command(x,t,p,c):
 crouch,push,fold,ret,bias,spin=c;q=np.array(p['qref']);u0=np.array(p['uref'])
 if t>=.18+push+ret:
  r=np.r_[q,np.zeros(6)];r[0]=bias*.5
  return np.clip(u0-np.array(p['K'])@(x[ix]-r[ix]),-limits,limits)
 h,hd,hdd=smooth(t,[(0,q[3]),(.18,q[3]+crouch),(.18+push,fold),(.18+push+ret,q[3])])
 ratio=q[4]/q[3];target=np.array([-bias if t<.18+push else 0,h,ratio*h]);vel=np.array([0,hd,ratio*hd]);acc=np.array([0,hdd,ratio*hdd]);desired=acc+120*(target-x[2:5])+22*(vel-x[8:11])
 old.load(scratch,x);mujoco.mj_forward(m,scratch)
 air=scratch.geom_xpos[wheel,2]>.055 and not any(wheel in (c.geom1,c.geom2) for c in scratch.contact)
 a0=scratch.qacc.copy();M=np.zeros((6,3))
 for j in range(3):
  scratch.ctrl[:]=0;scratch.ctrl[j]=1;scratch.qacc_warmstart[:]=0;mujoco.mj_forward(m,scratch);M[:,j]=scratch.qacc-a0
 weights=np.array([.08 if air else 1,1,1])/100
 def res(u):
  scratch.ctrl[:]=u;scratch.qacc_warmstart[:]=0;mujoco.mj_forward(m,scratch)
  return np.r_[(scratch.qacc[2:5]-desired)*weights,spin*(x[8:12].sum()+.03*scratch.qacc[2:6].sum()),spin*.1*u[2]*x[11]]
 guess=np.clip(np.linalg.lstsq(M[2:5],desired-a0[2:5],rcond=1e-6)[0],-limits+1e-8,limits-1e-8)
 return least_squares(res,guess,bounds=(-limits,limits),max_nfev=20,ftol=1e-5,xtol=1e-5,gtol=1e-5).x

def trace(p,c):
 x=np.r_[p['qref'],np.zeros(6)];xs=[x.copy()];us=[]
 for k in range(600):
  u=command(x,k*.01,p,c);x,_=old.step(x,u);xs.append(x);us.append(u)
  if abs(x[2])>.6 or x[1]<.15 or abs(x[3])>1.28 or abs(x[4])>2.53:break
 return np.array(xs),np.array(us)

def metrics(xs,us):
 flight=longest=0;pen=exc=rate=work=pitch=0.;path=[];landing=None;airSeen=False;landSpeed=None
 for x,u in zip(xs[1:],us):
  old.load(d,x,u);mujoco.mj_forward(m,d)
  pos=d.geom_xpos[wheel,[0,2]].copy();air=pos[1]>.055 and not any(wheel in (c.geom1,c.geom2) for c in d.contact)
  flight=flight+.01 if air else 0;longest=max(longest,flight)
  if air:airSeen=True;rate=max(rate,abs(x[8:12].sum()));work+=abs(u[2]*x[11])*.01
  elif airSeen and landing is None and any(wheel in (c.geom1,c.geom2)for c in d.contact):
   landing=pos.tolist();jac=np.zeros((3,6));mujoco.mj_jacGeom(m,d,jac,None,wheel);landSpeed=float((jac@x[6:])[0]-.05*x[8:12].sum())
  path.append(pos.tolist());pen=max(pen,max([0.]+[-float(c.dist)for c in d.contact]));exc=max(exc,0,abs(x[3])-1.26,abs(x[4])-2.51);pitch=max(pitch,abs(x[2]))
 apex=max(path,key=lambda v:v[1]);terminal=float(np.max(np.abs(xs[-50:,6:])))
 passed=len(us)==600 and longest>=.03 and apex[1]>=.07 and pen<=.01 and exc<=.02 and pitch<=.6 and terminal<=.3 and rate<=25
 return dict(passed=bool(passed),steps=len(us),flightS=longest,apex=apex,landing=landing,landingTangentialSpeed=landSpeed,airborneWheelRate=rate,airborneAbsoluteMotorWorkJ=work,penetration=pen,jointOverrun=exc,pitch=pitch,terminalRate=terminal,wheelPath=path[::5],basePath=xs[1::5,:2].tolist())

def main():
 start=time.perf_counter();out=ROOT/'test-results/target-jump-search.json';rows=[];best=[]
 (ROOT/'tests/fixtures/wheelbot_target_jump.json').write_text(json.dumps(PROTOCOL,indent=2)+'\n')
 for height in [.425,.3925,.4575]:
  p=min(atlas['profiles'],key=lambda p:abs(p['qref'][1]-height)+abs(p['qref'][2]))
  for c in itertools.product([.10,.18],[.18,.24,.30],[.32,.45],[.4],[0],[.02,.05]):
   xs,us=trace(p,c);r=metrics(xs,us);r.update(height=height,parameters=c);rows.append(r)
   print(json.dumps({k:v for k,v in r.items()if k!='wheelPath'}),flush=True)
   if r['passed']:
    best.append(dict(p=p,c=c,metrics=r,ref=xs.tolist(),u=us.tolist()));(ROOT/'test-results/target-jump-candidates.json').write_text(json.dumps(best))
   out.write_text(json.dumps(dict(protocol=PROTOCOL,seconds=time.perf_counter()-start,results=rows)))
   if len(best)>=2:return

def family():
 rows=[];selected=[];start=time.perf_counter()
 for height in [.3925,.425,.4575]:
  p=min(atlas['profiles'],key=lambda p:abs(p['qref'][1]-height)+abs(p['qref'][2]))
  for bias in [-.08,0,.08]:
   for crouch,push,fold in [( .18,.3,.32),(.1,.24,.45),(.18,.27,.4),(.18,.28,.32),(.18,.26,.45),(.1,.22,.45),(.18,.25,.45),(.18,.24,.5)]:
    c=(crouch,push,fold,.4,bias,.02);xs,us=trace(p,c);r=metrics(xs,us);r.update(height=height,parameters=c);rows.append(r)
    print(json.dumps({k:v for k,v in r.items()if k!='wheelPath'}),flush=True)
    if r['passed']:
     selected.append(dict(p=p,c=c,metrics=r,ref=xs.tolist(),u=us.tolist()));break
  (ROOT/'test-results/target-jump-family-candidates.json').write_text(json.dumps(selected))
  (ROOT/'test-results/target-jump-family-search.json').write_text(json.dumps(dict(protocol=PROTOCOL,seconds=time.perf_counter()-start,results=rows)))

def compile_family(candidate_file="target-jump-family-candidates.json"):
 from scipy.linalg import solve
 start=time.perf_counter();candidates=json.loads((ROOT/'test-results'/candidate_file).read_text());profiles=[]
 H=np.eye(11)[[0,1,2,3,4,10]];Qe=np.array(base['Qe']);Re=np.array(base['Re']);I=np.eye(11)
 for item in candidates:
  n=140;p=item['p'];xs=np.array(item['ref']);us=np.array(item['u']);As=[];Bs=[]
  for k in range(n):
   A=np.zeros((11,11));B=np.zeros((11,3))
   for kind,size,M in [('x',11,A),('u',3,B)]:
    for j in range(size):
     dx=np.zeros(12);du=np.zeros(3);eps=1e-7
     if kind=='x':dx[ix[j]]=eps
     else:du[j]=eps
     xp,_=old.step(xs[k]+dx,us[k]+du);xm,_=old.step(xs[k]-dx,us[k]-du);M[:,j]=(xp-xm)[ix]/(2*eps)
   As.append(A);Bs.append(B)
  Q=np.array(base['Q']);R=np.array(base['R']);P=np.array(p['P']);Ks=[None]*n
  for k in reversed(range(n)):
   A=As[k];B=Bs[k];K=solve(R+B.T@P@B,B.T@P@A,assume_a='pos');F=A-B@K;P=Q+K.T@R@K+F.T@P@F;P=(P+P.T)/2;Ks[k]=K
  L0=np.array(base['L']);Pe=np.array(base['kalmanPredictedCovariance']);Pe=(I-L0@H)@Pe@(I-L0@H).T+L0@Re@L0.T;Ls=[]
  for A in As:
   pred=A@Pe@A.T+Qe;L=solve(H@pred@H.T+Re,H@pred,assume_a='pos').T;Pe=(I-L@H)@pred@(I-L@H).T+L@Re@L.T;Pe=(Pe+Pe.T)/2;Ls.append(L)
  terminal={key:p[key]for key in ['qref','uref','K','A','B','L']};terminal['qref']=list(terminal['qref']);terminal['qref'][0]=item['c'][4]*.5
  pr=dict(id=len(profiles),parameters=item['c'],entry=xs[0].tolist(),ref=xs[:n+1].tolist(),u=us[:n].tolist(),A=np.array(As).tolist(),B=np.array(Bs).tolist(),K=np.array(Ks).tolist(),L=np.array(Ls).tolist(),terminal=terminal,referenceMetrics=item['metrics'])
  profiles.append(pr);print('compiled',pr['id'],item['metrics']['height'],item['c'][4],flush=True)
 bundle=dict(schema='wheelbot-target-jump/v1',assetSha256=hashlib.sha256((ROOT/'assets/wheelbot/live_model.xml').read_bytes()).hexdigest(),baselineSha256=hashlib.sha256((ROOT/'assets/wheelbot/live_profile.json').read_bytes()).hexdigest(),nativeVersion=mujoco.mj_versionString(),generation=base.get('generation'),protocol=PROTOCOL,controlDt=.01,physicsDt=.002,steps=600,scheduledSteps=140,controlledIndices=ix,measurementIndices=base['measurementIndices'],measurementSigma=base['measurementSigma'],limitsNm=limits.tolist(),externalForce=[0,0,0],entryTolerance=dict(height=.003,pitch=.006,joints=.008,rates=.03),profiles=profiles,releaseValidated=False,planning='finite checked family projection, not globally closest; translation in x only',offlineSeconds=time.perf_counter()-start)
 (ROOT/'assets/wheelbot/target_jump.json').write_text(json.dumps(bundle,separators=(',',':'),allow_nan=False)+'\n')

def expand():
 selected=json.loads((ROOT/'test-results/target-jump-family-candidates.json').read_text());rows=[]
 for height in [.4575,.425,.3925]:
  p=min(atlas['profiles'],key=lambda p:abs(p['qref'][1]-height)+abs(p['qref'][2]))
  for bias in [-.025,0,.025]:
   found=0
   params=[(.18,.24,.32),(.18,.25,.32),(.18,.26,.32),(.18,.24,.45),(.18,.22,.5),(.1,.23,.45),(.1,.25,.4),(.18,.29,.35)] if height==.4575 else [(.1,.24,.45),(.1,.25,.45),(.1,.26,.4),(.18,.26,.45),(.18,.28,.4),(.18,.29,.32),(.18,.3,.32)]
   for crouch,push,fold in params:
    c=(crouch,push,fold,.4,bias,.02);xs,us=trace(p,c);r=metrics(xs,us);r.update(height=height,parameters=c);rows.append(r)
    print(json.dumps({k:v for k,v in r.items()if k!='wheelPath'}),flush=True)
    if r['passed']:
     selected.append(dict(p=p,c=c,metrics=r,ref=xs.tolist(),u=us.tolist()));found+=1
     if found==2:break
   (ROOT/'test-results/target-jump-expanded-candidates.json').write_text(json.dumps(selected))
   (ROOT/'test-results/target-jump-expanded-search.json').write_text(json.dumps(dict(protocol=PROTOCOL,results=rows)))


def rebuild():
 existing=json.loads((ROOT/'assets/wheelbot/target_jump.json').read_text());selected=[]
 for pr in existing['profiles']:
  p=min(atlas['profiles'],key=lambda p:abs(p['qref'][1]-pr['entry'][1])+abs(p['qref'][2]))
  xs,us=trace(p,pr['parameters']);r=metrics(xs,us);r['height']=p['qref'][1]
  if not r['passed']:raise ValueError('Stored primitive no longer passes exact-state design')
  selected.append(dict(p=p,c=pr['parameters'],metrics=r,ref=xs.tolist(),u=us.tolist()))
 (ROOT/'test-results/target-jump-rebuild-candidates.json').write_text(json.dumps(selected))
 compile_family('target-jump-rebuild-candidates.json')

if __name__=='__main__':
 import argparse
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('mode',choices=['search','family','expand','rebuild'],default='rebuild',nargs='?');args=parser.parse_args()
 {'search':main,'family':family,'expand':expand,'rebuild':rebuild}[args.mode]()
