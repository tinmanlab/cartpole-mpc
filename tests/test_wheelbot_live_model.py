"""Independent native checks; never regenerates or writes controller gains."""
from pathlib import Path
import hashlib, json, math, time
import mujoco
import numpy as np
from scipy.linalg import solve_discrete_are
ROOT=Path(__file__).resolve().parents[1]
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path): return json.loads(path.read_text())
ACCEPTANCE={'durationS':10.,'finalWindowS':2.,'pitchErrorRad':.08,'jointErrorRad':.10,'positionErrorM':.02,'velocityComponentMax':.3,'jointLimitExcursionRad':.02,'penetrationM':.005}
SEEDS=[1,7,42,2026]
# Five cases per seed; live goals switch at 2 seconds without resetting state.
CASES=[{'pitch':0.,'hip':0.,'goal':0.},{'pitch':.02,'hip':.01,'goal':0.},{'pitch':-.02,'hip':-.01,'goal':0.},{'pitch':.02,'hip':-.01,'goal':.03},{'pitch':-.02,'hip':.01,'goal':-.03}]
def main():
 p=read(ROOT/'assets/wheelbot/live_profile.json'); old=read(ROOT/'assets/wheelbot/contact_profile.json')
 m=mujoco.MjModel.from_xml_path(str(ROOT/p['asset'])); d=mujoco.MjData(m)
 assert p['assetSha256']==sha(ROOT/p['asset'])
 assert p['schema']=='wheelbot-profile/v1' and p['controlDt']==.01 and p['substeps']==5
 np.testing.assert_allclose(p['qref'][3:5],[.55,-1.10],atol=1e-14)
 assert p['K']!=old['K'] and p['A']!=old['A'] and p['B']!=old['B']
 assert (m.nq,m.nv,m.nu,m.na,m.neq,m.ntendon,m.nmocap)==(6,6,3,0,0,0,0)
 assert m.opt.timestep==.002 and int(m.opt.disableflags)==0 and int(m.opt.enableflags)==0
 np.testing.assert_allclose(m.body_mass.sum(),1.19915,atol=1e-13)
 np.testing.assert_allclose(m.body_ipos[1],[0,0,.02],atol=1e-14)
 np.testing.assert_allclose(m.body_inertia[1],[(.07**2+.075**2)/3,(.085**2+.075**2)/3,(.085**2+.07**2)/3],atol=1e-14)
 for inertia in m.body_inertia[1:]: assert min(inertia)>0 and 2*max(inertia)<=sum(inertia)+1e-14
 np.testing.assert_array_equal(m.actuator_trnid[:,0],[3,4,5])
 np.testing.assert_allclose(m.actuator_gear[:,0],1)
 np.testing.assert_allclose(m.actuator_ctrlrange[:,1],[16.,16.,1.7])
 assert not m.body_gravcomp.any()
 for name in ['torso_visual','upper_link_visual','lower_link_visual','wheel_visual']:
  g=m.geom(name).id; assert m.geom_contype[g]==m.geom_conaffinity[g]==1
 assert m.geom('torso_visual').type==mujoco.mjtGeom.mjGEOM_BOX
 np.testing.assert_allclose(m.geom('torso_visual').size,[.085,.07,.075])
 assert mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'obstacle')==-1
 A,B,Q,R,K,L,Qe,Re=[np.array(p[k]) for k in ['A','B','Q','R','K','L','Qe','Re']]
 P=solve_discrete_are(A,B,Q,R); np.testing.assert_allclose(P,p['P'],rtol=1e-9,atol=1e-9)
 np.testing.assert_allclose(np.linalg.solve(R+B.T@P@B,B.T@P@A),K,rtol=1e-9,atol=1e-9)
 H=np.zeros((6,11));H[:5,:5]=np.eye(5);H[5,10]=1; Pe=solve_discrete_are(A.T,H.T,Qe,Re)
 np.testing.assert_allclose(np.linalg.solve(H@Pe@H.T+Re,H@Pe).T,L,rtol=1e-9,atol=1e-9)
 print('Compiled geometry, mass, inertia, transmission and independent Riccati/KF checks passed')
 qref=np.array(p['qref']); uref=np.array(p['uref']); ref=np.r_[qref[:5],np.zeros(6)]; idx=p['controlledIndices']; limits=np.array(p['limitsNm'])
 xref=np.r_[qref,np.zeros(6)]
 def step(x,u):
  mujoco.mj_resetData(m,d);d.qpos[:]=x[:6];d.qvel[:]=x[6:];d.ctrl[:]=u
  assert not d.qfrc_applied.any() and not d.xfrc_applied.any()
  mujoco.mj_step(m,d,nstep=5)
  return np.r_[d.qpos,d.qvel].copy()
 # Independently differentiate the actual reset + five-step map, not its one-step composition.
 eps=1e-6; fdA=np.zeros((11,11));fdB=np.zeros((11,3))
 for j,k in enumerate(idx):
  e=np.zeros(12);e[k]=eps;fdA[:,j]=(step(xref+e,uref)-step(xref-e,uref))[idx]/(2*eps)
 for j in range(3):
  e=np.zeros(3);e[j]=eps;fdB[:,j]=(step(xref,uref+e)-step(xref,uref-e))[idx]/(2*eps)
 map_error=max(float(np.max(abs(fdA-A))),float(np.max(abs(fdB-B))))
 assert map_error/max(1.,float(np.max(abs(A))))<1e-7,map_error
 phase=xref.copy();phase[5]=.713
 phase_error=float(np.max(abs((step(phase,uref)-phase)[idx]-(step(xref,uref)-xref)[idx])))
 assert phase_error<1e-9
 perturb=np.linspace(-1.,1.,11)*1e-6; xp=xref.copy();xp[idx]+=perturb
 prediction_error=float(np.max(abs(step(xp,uref)[idx]-ref-A@perturb)))
 assert prediction_error<1e-7
 obs=np.vstack([H@np.linalg.matrix_power(A,k) for k in range(11)])
 obs_s=np.linalg.svd(obs,compute_uv=False); obs_rank=int(np.linalg.matrix_rank(obs));assert obs_rank==11
 unstable=[v for v in np.linalg.eigvals(A) if abs(v)>=1-1e-9]
 pbh=[float(np.linalg.svd(np.column_stack([v*np.eye(11)-A,B]),compute_uv=False)[-1]) for v in unstable]
 assert min(pbh)>1e-8
 mujoco.mj_resetData(m,d);d.qpos[:]=qref;d.ctrl[:]=uref;mujoco.mj_forward(m,d)
 assert np.max(abs(d.qacc))<1e-8
 np.testing.assert_allclose(d.qfrc_actuator[:3],0,atol=1e-14)
 trim_com=d.subtree_com[1].copy();trim_wheel=d.site_xpos[m.site('wheel_site').id].copy()
 pairs=[(m.geom(a).id,m.geom(b).id) for a,b in [('torso_visual','lower_link_visual'),('torso_visual','wheel_visual'),('upper_link_visual','wheel_visual')]]
 trim_distances=[float(mujoco.mj_geomDistance(m,d,a,b,1.,None)) for a,b in pairs]
 assert min(trim_distances)>0
 trials=[];started=time.perf_counter()
 for seed in SEEDS:
  for case in CASES:
   rng=seed
   def uniform():
    nonlocal rng
    rng=(1664525*rng+1013904223)&0xffffffff
    return (rng+.5)/4294967296
   def measure(x):
    return x[[0,1,2,3,4,11]]+np.array(p['measurementSigma'])*np.array([math.sqrt(-2*math.log(uniform()))*math.cos(2*math.pi*uniform()) for _ in range(6)])
   x=xref.copy();x[2]+=case['pitch'];x[3]+=case['hip'];initial=x.copy()
   initial_measurement=measure(x);estimate=np.r_[initial_measurement[:5],np.zeros(6)]
   peak_pen=peak_exc=0.;min_distance=1.;peak_tau=np.zeros(3);tail=np.zeros(5);saturated=0;contact_loss=0;history=[];failed=False
   for k in range(1000):
    goal=case['goal'] if k>=200 else 0.
    target=ref.copy();target[0]+=goal
    requested=uref-K@(estimate-target);u=np.clip(requested,-limits,limits)
    saturated+=int(np.any(abs(requested)>limits));peak_tau=np.maximum(peak_tau,abs(u));before=x.copy()
    mujoco.mj_resetData(m,d);d.qpos[:]=x[:6];d.qvel[:]=x[6:];d.ctrl[:]=u
    assert not d.qfrc_applied.any() and not d.xfrc_applied.any()
    for substep in range(5):
     mujoco.mj_step(m,d)
     peak_pen=max(peak_pen,max([0.]+[-float(c.dist) for c in d.contact]))
     peak_exc=max(peak_exc,float(np.max(np.maximum(m.jnt_range[3:5,0]-d.qpos[3:5],d.qpos[3:5]-m.jnt_range[3:5,1]))))
    x=np.r_[d.qpos,d.qvel].copy();measurement=measure(x)
    prediction=ref+A@(estimate-ref)+B@(u-uref);estimate=prediction+L@(measurement-H@prediction)
    mujoco.mj_forward(m,d)
    min_distance=min(min_distance,*[float(mujoco.mj_geomDistance(m,d,a,b,1.,None)) for a,b in pairs])
    if not any(c.geom1==m.geom('wheel_visual').id or c.geom2==m.geom('wheel_visual').id for c in d.contact):contact_loss+=1
    if k>=800:
     tail=np.maximum(tail,[abs(x[2]-qref[2]),abs(x[3]-qref[3]),abs(x[4]-qref[4]),abs(x[0]-goal),np.max(abs(x[6:]))])
    history.append({'before':before.tolist(),'after':x.tolist(),'u':u.tolist(),'measurement':measurement.tolist(),'goal':goal})
    failed=not np.isfinite(x).all() or abs(x[0])>1 or abs(x[2]-qref[2])>.6 or x[1]<.12
    if failed:break
   checks={'balance10s':not failed and len(history)==1000,'pitch':bool(tail[0]<=.08),'hip':bool(tail[1]<=.10),'knee':bool(tail[2]<=.10),'position':bool(tail[3]<=.02),'velocity':bool(tail[4]<=.3),'torque':bool(np.all(peak_tau<=limits)),'jointLimits':bool(peak_exc<=.02),'penetration':bool(peak_pen<=.005)}
   metrics={'passed':all(checks.values()),'checks':checks,'finalWindowMaxErrors':dict(zip(['pitchRad','hipRad','kneeRad','positionM','velocityComponent'],tail.tolist())),'finalWindowMaxVelocityComponents':np.max(np.abs(np.array([row['after'][6:] for row in history[-200:]])),axis=0).tolist(),'peakTorqueNm':peak_tau.tolist(),'maximumPenetrationM':peak_pen,'maximumJointExcursionRad':peak_exc,'minimumNonadjacentDistanceM':min_distance,'saturatedCommands':saturated,'wheelContactLostCommands':contact_loss}
   trials.append({'seed':seed,'case':case,'initial':initial.tolist(),'initialMeasurement':initial_measurement.tolist(),'metrics':metrics,'trace':history})
 rollout_seconds=time.perf_counter()-started
 protocol=read(ROOT/'assets/wheelbot/live_design.json'); assert protocol['acceptance']==ACCEPTANCE
 assert protocol['profileSha256']==sha(ROOT/'assets/wheelbot/live_profile.json') and protocol['modelSha256']==sha(ROOT/p['asset'])
 report={'schema':'wheelbot-live-model-evidence/v1','modelSha256':protocol['modelSha256'],'profileSha256':protocol['profileSha256'],'mujocoVersion':mujoco.__version__,'acceptance':ACCEPTANCE,'trials':len(trials),'passed':sum(t['metrics']['passed'] for t in trials),'failed':sum(not t['metrics']['passed'] for t in trials),'torquePassCount':sum(t['metrics']['checks']['torque'] for t in trials),'rolloutSeconds':rollout_seconds,'actualMapFDMaxAbs':map_error,'predictionMaxAbs':prediction_error,'phasePredictionMaxAbs':phase_error,'observabilityRank':obs_rank,'observabilitySingularValues':obs_s.tolist(),'unstableModePBHMinSingularValues':pbh,'trimCOMWorldM':trim_com.tolist(),'trimWheelWorldM':trim_wheel.tolist(),'trimNonadjacentDistancesM':trim_distances,'qref':p['qref'],'uref':p['uref'],'closedLoopPoles':[[float(z.real),float(z.imag)] for z in np.linalg.eigvals(A-B@K)],'closedLoopRadius':p['closedLoopRadius'],'observerErrorRadius':p['observerErrorRadius'],'results':[{k:v for k,v in t.items() if k not in ['trace','initialMeasurement','initial']} for t in trials],'scope':protocol['scope']}
 out=ROOT/'test-results';out.mkdir(exist_ok=True)
 def atomic(path,value):
  tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(value,allow_nan=False)+'\n');tmp.replace(path)
 report['gainSha256']=hashlib.sha256(json.dumps(p['K'],separators=(',',':')).encode()).hexdigest()
 report['weightChoice']=p['weightChoice']
 report['checkPassCounts']={key:sum(t['metrics']['checks'][key] for t in trials) for key in trials[0]['metrics']['checks']}
 report['trimQaccInf']=p['trimQaccInf'];report['dareNormalizedResidual']=p['dareNormalizedResidual']
 report['actualMapFDRelativeMax']=map_error/max(1.,float(np.max(abs(A))))
 js_path=out/'live-model-js-replay.json'
 if js_path.exists():
  replay=read(js_path)
  if replay['profileSha256']==report['profileSha256']:report['jsControllerReplay']=replay
 write_start=time.perf_counter();atomic(out/'live-model-reference.json',{'schema':'wheelbot-live-native-reference/v1','modelSha256':protocol['modelSha256'],'profileSha256':protocol['profileSha256'],'controlDt':.01,'substeps':5,'noise':'JS uint32 LCG 1664525/1013904223 + Box-Muller; five noisy pose channels plus simulated wheel encoder rate','trials':trials});report['rawWriteSeconds']=time.perf_counter()-write_start
 atomic(ROOT/'evidence/wheelbot_live_model.json',report)
 print(json.dumps({k:v for k,v in report.items() if k not in ['results','closedLoopPoles','observabilitySingularValues']},indent=2))
 print('Check pass counts:',report['checkPassCounts'])
 assert report['passed']==20, f"{report['passed']}/20 acceptance passes; inspect evidence"

if __name__=='__main__': main()
