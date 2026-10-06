"""Compact-model jump reference and TVLQR/KF design.
Reuses the established repository nonlinear reference + finite-horizon Riccati
method. Real three-motor actuation; no added base force or model replacement.
"""
from pathlib import Path
import json,time,hashlib
import numpy as np
import mujoco
from scipy.optimize import least_squares
from scipy.linalg import solve
ROOT=Path(__file__).resolve().parents[1]
m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/wheelbot/live_model.xml'))
p=json.loads((ROOT/'assets/wheelbot/live_profile.json').read_text())
d=mujoco.MjData(m);scratch=mujoco.MjData(m);q0=np.array(p['qref']);u0=np.array(p['uref']);limits=m.actuator_ctrlrange[:,1].copy();K=np.array(p['K']);idx=p['controlledIndices'];ref=np.r_[q0,np.zeros(6)]

def load(data,x,u=None):
 mujoco.mj_resetData(m,data);data.qpos[:]=x[:6];data.qvel[:]=x[6:]
 if u is not None:data.ctrl[:]=u

def posture(t,extension):
 points=[(0.,.55),(.15,.55),(.7,.90),(.7+extension,.20),(1.1+extension,.55)]
 if t>=points[-1][0]:return .55,0.,0.,'settle'
 for (t0,h0),(t1,h1) in zip(points,points[1:]):
  if t<t1:
   f=max(0.,(t-t0)/(t1-t0));span=t1-t0;s=10*f**3-15*f**4+6*f**5;ds=(30*f*f-60*f**3+30*f**4)/span;dds=(60*f-180*f*f+120*f**3)/span**2
   return h0+(h1-h0)*s,(h1-h0)*ds,(h1-h0)*dds,'motion'

def command(x,t,extension):
 h,hd,hdd,phase=posture(t,extension)
 if phase=='settle' and abs(x[2]-q0[2])<.4 and x[1]>.3:
  return np.clip(u0-K@(x[idx]-ref[idx]),-limits,limits)
 target=np.array([q0[2]-.5*(x[0]+.5*x[6]),h,-2*h]);vel=np.array([0,hd,-2*hd]);acc=np.array([0,hdd,-2*hdd]);desired=acc+120*(target-x[2:5])+22*(vel-x[8:11])
 load(scratch,x);mujoco.mj_forward(m,scratch);a0=scratch.qacc[2:5].copy();map=np.zeros((3,3))
 for j in range(3):
  scratch.ctrl[:]=0;scratch.ctrl[j]=1;scratch.qacc_warmstart[:]=0;mujoco.mj_forward(m,scratch);map[:,j]=scratch.qacc[2:5]-a0
 guess=np.clip(np.linalg.lstsq(map,desired-a0,rcond=1e-6)[0],-limits+1e-8,limits-1e-8)
 def residual(u):
  scratch.ctrl[:]=u;scratch.qacc_warmstart[:]=0;mujoco.mj_forward(m,scratch);return scratch.qacc[2:5]-desired
 result=least_squares(residual,guess,bounds=(-limits,limits),max_nfev=12,ftol=1e-5,gtol=1e-5,xtol=1e-5)
 return result.x


def step(x,u):
 load(d,x,u)
 for _ in range(5):mujoco.mj_step(m,d)
 return np.r_[d.qpos,d.qvel].copy(),tuple((int(c.geom1),int(c.geom2))for c in d.contact)

def metrics(states,inputs):
 flight=longest=0;clear=0.;pen=0.;exc=0.;obs=[];xs=np.array(states)
 for x,u in zip(states,[inputs[0]]+list(inputs)):
  load(d,x,u);mujoco.mj_forward(m,d);wheel=m.geom('wheel_visual').id
  n=sum(wheel in (c.geom1,c.geom2)for c in d.contact);height=float(d.geom_xpos[wheel,2]-.05)
  flight=flight+1 if n==0 and height>.005 else 0;longest=max(longest,flight);clear=max(clear,height)
  pen=max(pen,max([0.]+[-float(c.dist)for c in d.contact]));exc=max(exc,0.,abs(x[3])-1.26,abs(x[4])-2.51)
  obs.append(float(d.subtree_com[1,2]))
 tail=xs[-50:];errs=np.max(np.abs(tail[:,:5]-q0[:5]),axis=0);rate=float(np.max(np.abs(tail[:,6:])))
 checks={'flight':longest*.01>=.03,'clearance':clear>=.005,'settledPosition':errs[0]<=.03,'settledPitch':errs[2]<=.04,'settledJoints':max(errs[3:5])<=.06,'settledSpeed':rate<=.3,'jointLimits':exc<=.02,'penetration':pen<=.01,'torques':bool(np.all(np.abs(inputs)<=limits)),'bodyEnvelope':bool(np.max(np.abs(xs[:,2]-q0[2]))<.6 and np.min(xs[:,1])>.15)}
 return {'passed':all(checks.values()),'checks':{k:bool(v)for k,v in checks.items()},'longestFlightSeconds':longest*.01,'wheelClearanceM':clear,'comApexM':max(obs)-obs[0],'maxPenetrationM':pen,'maxJointExcursionRad':exc,'tailErrors':errs.tolist(),'tailMaxRate':rate}

def main():
 start=time.perf_counter();states=[ref.copy()];inputs=[]
 for k in range(600):
  u=command(states[-1],k*.01,.22);inputs.append(u);states.append(step(states[-1],u)[0])
 validation=metrics(states,inputs)
 if not validation['passed']:raise ValueError('Actual compact jump rejected: '+json.dumps(validation))
 As=[];Bs=[];crossings=0;errors=[]
 for k in range(600):
  x=states[k];u=inputs[k];A=np.zeros((11,11));B=np.zeros((11,3))
  for kind,size,M in [('x',11,A),('u',3,B)]:
   for j in range(size):
    pair=[]
    for eps in [1e-7,1e-8]:
     dx=np.zeros(12);du=np.zeros(3)
     if kind=='x':dx[idx[j]]=eps
     else:du[j]=eps
     xp,cp=step(x+dx,u+du);xm,cm=step(x-dx,u-du);pair.append(((xp-xm)[idx]/(2*eps),cp==cm))
    selected=pair[0]if pair[0][1]else pair[1]
    if not selected[1]or not np.isfinite(selected[0]).all():raise ValueError(f'Nonlocal jump derivative {k}/{kind}/{j}')
    crossings+=int(not pair[0][1]);errors.append(float(np.max(np.abs(pair[0][0]-pair[1][0]))/max(1.,float(np.max(np.abs(pair[1][0]))))))
    M[:,j]=selected[0]
  As.append(A);Bs.append(B)
 Q=np.array(p['Q']);R=np.array(p['R']);P=np.array(p['P']);Ks=[None]*600
 for k in reversed(range(600)):
  A=As[k];B=Bs[k];gain=solve(R+B.T@P@B,B.T@P@A,assume_a='pos');F=A-B@gain;P=Q+gain.T@R@gain+F.T@P@F;P=(P+P.T)/2;Ks[k]=gain
 H=np.array(p['measurementModel']);Qe=np.array(p['Qe']);Re=np.array(p['Re']);L0=np.array(p['L']);Pe=np.array(p['kalmanPredictedCovariance']);I=np.eye(11);Pe=(I-L0@H)@Pe@(I-L0@H).T+L0@Re@L0.T;Ls=[]
 for A in As:
  pred=A@Pe@A.T+Qe;L=solve(H@pred@H.T+Re,H@pred,assume_a='pos').T;Pe=(I-L@H)@pred@(I-L@H).T+L@Re@L.T;Pe=(Pe+Pe.T)/2;Ls.append(L)
 profile={'schema':'wheelbot-action-jump/v1','assetSha256':hashlib.sha256((ROOT/'assets/wheelbot/live_model.xml').read_bytes()).hexdigest(),'baselineSha256':hashlib.sha256((ROOT/'assets/wheelbot/live_profile.json').read_bytes()).hexdigest(),'generatorSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'controlDt':.01,'physicsDt':.002,'steps':600,'controlledIndices':idx,'measurementIndices':[0,1,2,3,4,11],'measurementSigma':p['measurementSigma'],'limitsNm':limits.tolist(),'externalForce':[0,0,0],'ref':[x.tolist()for x in states],'u':[u.tolist()for u in inputs],'A':[a.tolist()for a in As],'B':[b.tolist()for b in Bs],'K':[k.tolist()for k in Ks],'L':[l.tolist()for l in Ls],'referenceMetrics':validation,'referenceDesign':{'extensionSeconds':.22,'crouchHipRad':.90,'extensionHipRad':.20,'settlingControllerStartSample':200,'developmentReason':'Preserve the0.22sphysical jump and the unchanged settling error/velocity limits; allow6stotal including measured postlandingbraking instead of declaring4stailassettled.'},'derivativeCheck':{'coarseContactCrossings':crossings,'maximumRefinementRelativeError':max(errors)},'entry':{'heightErrorM':.008,'pitchErrorRad':.025,'jointErrorRad':.04,'speedLimit':.15},'claim':'Known compact-model jump trajectory, no arbitrary fallen recovery, source reference uses exact state offline; runtime uses the same six noisy measurements.','offlineSeconds':time.perf_counter()-start}
 (ROOT/'assets/wheelbot/action_jump.json').write_text(json.dumps(profile,separators=(',',':'),allow_nan=False)+'\n')
 (ROOT/'test-results/action-jump-design.json').write_text(json.dumps({'metrics':validation,'offlineSeconds':profile['offlineSeconds'],'derivatives':profile['derivativeCheck']},indent=2)+'\n')
 print(json.dumps({'metrics':validation,'offlineSeconds':profile['offlineSeconds'],'derivatives':profile['derivativeCheck']},indent=2))

if __name__=='__main__':main()
