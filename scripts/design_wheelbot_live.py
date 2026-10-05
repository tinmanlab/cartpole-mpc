"""Design a local planar wheelbot LQR/KF profile with native MuJoCo FD + SciPy."""
from __future__ import annotations
import hashlib, json
from pathlib import Path
import mujoco, numpy as np, scipy, scipy.linalg as la
from scipy.optimize import least_squares

ROOT=Path(__file__).resolve().parents[1]
XML=ROOT/'assets/wheelbot/live_model.xml'
OUT=ROOT/'assets/wheelbot/live_profile.json'
DT=.01
CONTROLLED=np.array([0,1,2,3,4,6,7,8,9,10,11])
MEASURED=5

def linearize(qref,uref,phase=0.):
    m=mujoco.MjModel.from_xml_path(str(XML)); d=mujoco.MjData(m)
    d.qpos[:]=qref; d.qpos[5]=phase; d.ctrl[:]=uref
    mujoco.mj_forward(m,d)
    F=np.zeros((12,12));G=np.zeros((12,3))
    mujoco.mjd_transitionFD(m,d,1e-6,True,F,G,None,None)
    a=np.eye(12);b=np.zeros((12,3))
    # Compose five native Euler steps to the browser's 10 ms control interval.
    for _ in range(5): b=F@b+G; a=F@a
    return a[np.ix_(CONTROLLED,CONTROLLED)],b[CONTROLLED,:],d

def design(*, hip=.55, knee=-1.10, initial_height=None):
    m=mujoco.MjModel.from_xml_path(str(XML));d=mujoco.MjData(m)
    # Declared crouched posture; solve root height/pitch and all three feedforward
    # torques from native acceleration, retaining the full precision solution.
    def residual(v):
        mujoco.mj_resetData(m,d)
        d.qpos[:]=[0.,v[0],v[1],hip,knee,0.]
        d.ctrl[:]=v[2:]
        mujoco.mj_forward(m,d)
        return d.qacc.copy()
    seed_height=.05+.25*(np.cos(hip)+np.cos(hip+knee))-.00025 if initial_height is None else float(initial_height)
    result=least_squares(residual,[seed_height,.01,.001,9.81*.25*np.sin(hip),.0001],
                         xtol=1e-14,ftol=1e-14,gtol=1e-14,max_nfev=1000)
    trim=float(np.max(np.abs(residual(result.x))))
    qref=d.qpos.copy();uref=d.ctrl.copy()
    if not result.success or trim>1e-8: raise ValueError(f'Trim rejected: {trim}')
    A,B,_=linearize(qref,uref)
    # State is [x,z,pitch,hip,knee, vx,vz, pitch_rate,hip_rate,knee_rate,wheel_rate].
    prior=json.loads((ROOT/'assets/wheelbot/contact_profile.json').read_text())
    Q=np.array(prior['Q']); Q[0,0]=80. # Stronger position cost for short live commands; not an optimum.
    R=np.diag([.08,.08,.35])
    P=la.solve_discrete_are(A,B,Q,R)
    K=la.solve(R+B.T@P@B,B.T@P@A)
    dare=la.norm(A.T@P@A-P-A.T@P@B@K+Q,np.inf)/max(1.,la.norm(P,np.inf))
    radii=np.abs(la.eigvals(A-B@K))
    H=np.zeros((6,11));H[np.arange(5),np.arange(5)]=1;H[5,10]=1
    measurement_sigma=np.array([.001,.001,.002,.003,.003,.02])
    process=np.diag([1e-8,1e-8,2e-7,2e-7,2e-7, 2e-6,2e-6,2e-5,2e-5,2e-5,5e-5])
    process*=.01 # Quiet local model: suppress sensor-driven wheel jitter; revisit for unmodeled disturbances.
    meas_cov=np.diag(measurement_sigma**2)
    Ppred=la.solve_discrete_are(A.T,H.T,process,meas_cov)
    L=la.solve(H@Ppred@H.T+meas_cov,H@Ppred).T
    observer_radius=float(max(abs(la.eigvals((np.eye(11)-L@H)@A))))
    phase_A,phase_B,_=linearize(qref,uref,phase=.713)
    phase_invariance=float(max(np.max(np.abs(phase_A-A)),np.max(np.abs(phase_B-B))))
    return {'schema':'wheelbot-profile/v1','model':'Compact crouched educational wheelbot; declared engineering approximation, not calibrated hardware',
      'asset':'assets/wheelbot/live_model.xml','assetSha256':hashlib.sha256(XML.read_bytes()).hexdigest(),'xmlSha256':hashlib.sha256(XML.read_bytes()).hexdigest(),
      'timestep':.002,'controlDt':DT,'substeps':5,'qOrder':['x','z','pitch','hip','knee','wheel'],'vOrder':['x_dot','z_dot','pitch_dot','hip_dot','knee_dot','wheel_dot'],
      'actuators':['hip_motor','knee_motor','wheel_motor'],'actuatorUnits':'N m','qref':qref.tolist(),'uref':uref.tolist(),
      'stateOrder':'[x,z,pitch,hip,knee,x_dot,z_dot,pitch_dot,hip_dot,knee_dot,wheel_dot] relative to qref/uref; wheel absolute phase omitted by axial rotational symmetry',
      'controlledIndices':CONTROLLED.tolist(),'measurementOrder':['x','z','pitch','hip','knee','wheel_rate'],'measurementIndices':[0,1,2,3,4,11],'measurementUnits':['m','m','rad','rad','rad','rad/s'],'measurementModel':H.tolist(),'sensorScope':'Five noisy pose channels plus an explicitly simulated wheel encoder-rate channel (sigma0.02rad/s); hardware calibration not established',
      'Qe':process.tolist(),'Re':meas_cov.tolist(),'kalmanPredictedCovariance':Ppred.tolist(),'measurementSigma':measurement_sigma.tolist(),'A':A.tolist(),'B':B.tolist(),'Q':Q.tolist(),'R':R.tolist(),'P':P.tolist(),'K':K.tolist(),'L':L.tolist(),
      'trimSolver':f'scipy.optimize.least_squares/native qacc; fixed hip={hip},knee={knee} rad', 'trimEvaluations':result.nfev,'trimQaccInf':trim,'dareNormalizedResidual':float(dare),'closedLoopRadius':float(max(radii)),
      'observerErrorRadius':observer_radius,'wheelPhaseInvarianceMaxAbs':phase_invariance,
      'designAvailable':bool(trim<1e-8 and dare<1e-8 and max(radii)<1 and observer_radius<1 and phase_invariance<1e-7),
      'modelMetadata':{**METADATA,'torsoMassKg':1.0,'eachLinkMassKg':.056,'wheelTireMassKg':.08715,'totalMassKg':float(m.body_mass.sum()),'linkLengthM':.25,'wheelRadiusM':.05,
       'derivedFrom':'Original source-derived Upkie/sketch leg and tire dimensions/masses; new rounded base is an engineering assumption, with 1 kg lumped body/drive assembly and no identified motor assemblies or remote transmission'},
      'weightChoice':{'prior':'assets/wheelbot/contact_profile.json','changed':'Q[0,0] from 2 to 80 for faster small position commands','optimalityClaim':False,'observerProcessCovarianceScale':.01,'observerReason':'One bounded revision after prior Qe gave 0.62–0.92 rad/s noisy wheel jitter; same measurementSigma. Quiet local-model assumption, not calibrated disturbance robustness.'},'limitsNm':[16.,16.,1.7],'solver':'scipy.linalg.solve_discrete_are; A/B from MuJoCo mjd_transitionFD and 5-step composition',
      'scope':'local design about solved bent-knee standing contact; no global or hardware stability claim'}

METADATA={
 'baseGeometryType':'ellipsoid','baseHalfSizeM':[.085,.07,.075],
 'baseSizeM':[.17,.14,.15], 'bodyCOMLocalM':[0.,0.,.02],
 'hipPivotLocalM':[0.,0.,0.], 'hipToBodyCOMDistanceM':.02,
 'postureAnglesDeg':{'hip':float(np.degrees(.55)),'knee':float(np.degrees(-1.10)),'kneeFlexion':float(np.degrees(1.10))},
 'massAssumptions':'1 kg lumped body/drive assembly + two 0.056 kg rods + 0.08715 kg wheel/tire; not hardware identified',
 'baseInertiaKgM2':[(.07**2+.075**2)/5,(.085**2+.075**2)/5,(.085**2+.07**2)/5],
 'fullBodyContact':True,'collisionExclusion':'Standard MuJoCo parent-child filter only',
 'designChoice':'Hip near COM is a bounded compact-base choice, not a universal stability theorem'}

def atomic_json(path,value):
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');tmp.replace(path)

def build_model():
    import xml.etree.ElementTree as ET
    root=ET.parse(ROOT/'assets/wheelbot/contact_model.xml').getroot()
    root.set('model','planar-wheelbot-live-crouch')
    world=root.find('worldbody'); world.remove(world.find("geom[@name='obstacle']"))
    torso=world.find("body[@name='torso']")
    torso.find('inertial').set('pos','0 0 0.02')
    torso.find('inertial').set('diaginertia',' '.join(map(str,METADATA['baseInertiaKgM2'])))
    geom=torso.find("geom[@name='torso_visual']")
    geom.set('type','ellipsoid');geom.set('pos','0 0 0.02');geom.set('size','0.085 0.07 0.075')
    root.find('keyframe/key').set('qpos',f'0 {.05+.5*np.cos(.55)} 0 .55 -1.10 0')
    tmp=XML.with_suffix('.xml.tmp'); ET.ElementTree(root).write(tmp,encoding='unicode');tmp.replace(XML)

if __name__=='__main__':
    build_model()
    profile=design();atomic_json(OUT,profile)
    protocol={'schema':'wheelbot-live-design/v1','modelSha256':profile['xmlSha256'],'profileSha256':hashlib.sha256(OUT.read_bytes()).hexdigest(),
      'modelMetadata':profile['modelMetadata'],'defaultMode':'lqr_kf','controlDt':.01,'substeps':5,
      'acceptance':{'durationS':10.,'finalWindowS':2.,'pitchErrorRad':.08,'jointErrorRad':.10,'positionErrorM':.02,'velocityComponentMax':.3,'jointLimitExcursionRad':.02,'penetrationM':.005},
      'seeds':[1,7,42,2026],'goalsM':[0.,.03,-.03],'goalSwitchTimeS':2.,'goalRampRequired':False,
      'scope':'Local crouched balance and small live commands only; no get-up, hardware or calibrated robustness claim'}
    atomic_json(ROOT/'assets/wheelbot/live_design.json',protocol)
    print(json.dumps({k:profile[k] for k in ['xmlSha256','qref','uref','trimQaccInf','dareNormalizedResidual','closedLoopRadius','observerErrorRadius','wheelPhaseInvarianceMaxAbs','designAvailable']},indent=2))
