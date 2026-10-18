"""Bounded contact-aware hardware/control feasibility comparison.

Full state is deliberately privileged in this OFFLINE study. A failed local
search is not a proof of physical impossibility. No candidate is auto-promoted.
"""
from pathlib import Path
import copy,hashlib,json,subprocess,time
import numpy as np
import mujoco
from scipy.optimize import differential_evolution

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'test-results/selfright-study';OUT.mkdir(parents=True,exist_ok=True)
base=json.loads((ROOT/'assets/wheelbot/contact_design.json').read_text());base['obstacle']['enabled']=False
variants=[('full-contact-original',copy.deepcopy(base)),('lower-base',copy.deepcopy(base)),('compact-damped',copy.deepcopy(base))]
variants[1][1]['base']['sizeM'][2]=.16
variants[2][1]['base']['sizeM'][2]=.16
variants[2][1]['upper']['lengthM']=.20;variants[2][1]['lower']['lengthM']=.20
variants[2][1]['motor']['dampingNms']=.04;variants[2][1]['motor']['armatureKgM2']=.0005
bounds=[(-1.2,1.2),(-2.4,2.4)]*3+[(-15,15)]
reports=[];start_all=time.perf_counter()
for name,c in variants:
 config=OUT/(name+'.json');xml=OUT/(name+'.xml');profile=OUT/(name+'-profile.json');config.write_text(json.dumps(c,indent=2)+'\n')
 subprocess.run(['node','scripts/build_wheelbot_contact.mjs',str(config.relative_to(ROOT)),str(xml.relative_to(ROOT))],cwd=ROOT,check=True,capture_output=True)
 subprocess.run([__import__('sys').executable,'scripts/design_wheelbot_contact_profile.py','--model',str(xml.relative_to(ROOT)),'--output',str(profile.relative_to(ROOT))],cwd=ROOT,check=True,capture_output=True)
 p=json.loads(profile.read_text());m=mujoco.MjModel.from_xml_path(str(xml));d=mujoco.MjData(m);limits=np.array(p['limitsNm']);K=np.array(p['K']);qref=np.array(p['qref']);u0=np.array(p['uref']);idx=p['controlledIndices'];rref=np.r_[qref,np.zeros(6)][idx]
 def step(x,u):
  mujoco.mj_resetData(m,d);d.qpos[:]=x[:6];d.qvel[:]=x[6:];d.ctrl[:]=u;mujoco.mj_step(m,d,nstep=5);return np.r_[d.qpos,d.qvel].copy()
 x=np.r_[[0.,c['upper']['lengthM']+c['lower']['lengthM']+.2,1.1,.15,-.3,0],np.zeros(6)]
 for _ in range(400):x=step(x,[0,0,0])
 fallen=x.copy();evaluations=[]
 def rollout(z,record=False):
  x=fallen.copy();target=rref.copy();switched=None;hist=[];max_violation=0.;max_penetration=0.
  times=[0.,.6,1.3,2.1,3.];hips=[fallen[3],z[0],z[2],z[4],qref[3]];knees=[fallen[4],z[1],z[3],z[5],qref[4]]
  for k in range(600):
   xhat=x[idx].copy();xhat[2]=np.arctan2(np.sin(xhat[2]),np.cos(xhat[2]))
   if switched is None and x[1]>.82*qref[1] and abs(xhat[2]-qref[2])<.25 and abs(x[3]-qref[3])<.35 and abs(x[4]-qref[4])<.5:switched=k;target[0]=x[0]
   if switched is not None:u=u0-K@(xhat-target)
   else:
    qtarget=np.array([np.interp(k*.01,times,hips),np.interp(k*.01,times,knees)])
    u=np.r_[3*(qtarget-x[3:5])-.18*x[9:11],.05*(z[6]-x[11])]
   u=np.clip(u,-limits,limits);x=step(x,u)
   violation=max(0,abs(x[3])-1.26,abs(x[4])-2.51);max_violation=max(max_violation,violation)
   if d.ncon:max_penetration=max(max_penetration,max(-float(d.contact[j].dist) for j in range(d.ncon)))
   hist.append(np.r_[x,u])
   if not np.isfinite(x).all()or abs(x[0]-fallen[0])>3:break
  a=np.array(hist);tail=a[-100:];pitch=np.arctan2(np.sin(tail[:,2]),np.cos(tail[:,2]));complete=len(a)==600
  passed=bool(complete and max_violation<.05 and max_penetration<.02 and np.min(tail[:,1])>.90*qref[1] and np.max(np.abs(pitch-qref[2]))<.08 and np.max(np.abs(tail[:,3:5]-qref[3:5]))<.12 and np.max(np.abs(tail[:,6:11]))<.3)
  score=(0 if complete else 100)+50*np.mean(((tail[:,1]-qref[1])/qref[1])**2)+10*np.mean((pitch-qref[2])**2)+np.mean((tail[:,3]-qref[3])**2+(tail[:,4]-qref[4])**2)+.02*np.mean(np.minimum(tail[:,6:11]**2,100))+100*max_violation**2+100*max(0,max_penetration-.01)
  return {'score':float(score),'passed':passed,'steps':len(a),'localControllerHandoverStep':switched,'maximumJointLimitExcursionRad':max_violation,'maximumContactPenetrationM':max_penetration,'maximumRootHeightM':float(np.max(a[:,1])),'minimumPitchErrorRad':float(np.min(np.abs(np.arctan2(np.sin(a[:,2]),np.cos(a[:,2]))-qref[2]))),'final':x.tolist(),'trace':a.tolist()if record else None}
 def objective(z):
  r=rollout(z);evaluations.append({'parameters':z.tolist(),**{k:v for k,v in r.items()if k!='trace'}});return r['score']
 start=time.perf_counter();r=differential_evolution(objective,bounds,popsize=4,maxiter=6,seed=902,polish=False,tol=0,atol=0,workers=1);result=rollout(r.x,True)
 report={'name':name,'configuration':c,'modelSha256':hashlib.sha256(xml.read_bytes()).hexdigest(),'profile':{k:p[k]for k in ['qref','uref','trimQaccInf','closedLoopRadius','designAvailable']},'evaluations':len(evaluations),'passedCandidates':sum(x['passed']for x in evaluations),'bestParameters':r.x.tolist(),'initialFallen':fallen.tolist(),'result':{k:v for k,v in result.items()if k!='trace'},'elapsedS':time.perf_counter()-start}
 (OUT/(name+'-raw.json')).write_text(json.dumps({'report':report,'bestTrace':result['trace'],'evaluations':evaluations},indent=2)+'\n');reports.append(report);print(json.dumps(report),flush=True)
summary={'schema':'wheelbot-selfright-feasibility/v1','sourceSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'nativeMuJoCoVersion':mujoco.__version__,'method':'Same 7-parameter joint-sequence and wheel-rate-target search budget, actual full-contact MuJoCo, SciPy differential_evolution; local standing LQR only after admissible handover','bounds':bounds,'acceptance':'600 control samples; final1s rootheight>90%newtrim, pitcherror<0.08rad, hip/kneeerror<0.12rad, speeds<0.3 in listed units; joint limit excursion<0.05rad, contact penetration<0.02m. Declared simulation acceptance, not hardware safety.','privilegedState':True,'groundTruthUsedFor':'Offline feasibility feedback/damping and evaluation only; not an implemented observer-controlled get-up','externalForces':False,'historyResetWithinTrial':False,'automaticRedesignPromotion':False,'trials':reports,'elapsedS':time.perf_counter()-start_all,'interpretation':'No local-search failure proves physical impossibility. Geometry/actuator edits and the reusable local profile are available, but a stable get-up must satisfy the declared physical and closed-loop gates before deployment.'}
(ROOT/'evidence/wheelbot_selfright_study.json').write_text(json.dumps(summary,indent=2)+'\n')
