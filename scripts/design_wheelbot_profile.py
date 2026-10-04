"""Design a local planar wheelbot LQR/KF profile with native MuJoCo FD + SciPy."""
from __future__ import annotations
import hashlib, json
from pathlib import Path
import mujoco, numpy as np, scipy, scipy.linalg as la
from scipy.optimize import least_squares

ROOT=Path(__file__).resolve().parents[1]
XML=ROOT/'assets/wheelbot/wheelbot.xml'
OUT=ROOT/'assets/wheelbot/profile.json'
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

def design():
    m=mujoco.MjModel.from_xml_path(str(XML));d=mujoco.MjData(m)
    # Declared modest bend; solve root height/pitch and all three feedforward
    # torques from native acceleration, retaining the full precision solution.
    def residual(v):
        mujoco.mj_resetData(m,d)
        d.qpos[:]=[0.,v[0],v[1],.15,-.3,0.]
        d.ctrl[:]=v[2:]
        mujoco.mj_forward(m,d)
        return d.qacc.copy()
    result=least_squares(residual,[.05+.5*np.cos(.15)-.0001,0.,0.,.4,0.],
                         xtol=1e-14,ftol=1e-14,gtol=1e-14,diff_step=1e-5,max_nfev=1000)
    trim=float(np.max(np.abs(residual(result.x))))
    qref=d.qpos.copy();uref=d.ctrl.copy()
    if not result.success or trim>1e-8: raise ValueError(f'Trim rejected: {trim}')
    A,B,_=linearize(qref,uref)
    # State is [x,z,pitch,hip,knee, vx,vz, pitch_rate,hip_rate,knee_rate,wheel_rate].
    Q=np.diag([2.,30.,80.,15.,15., .6,8.,4.,1.5,1.5,.25])
    R=np.diag([.08,.08,.35])
    P=la.solve_discrete_are(A,B,Q,R)
    K=la.solve(R+B.T@P@B,B.T@P@A)
    dare=la.norm(A.T@P@A-P-A.T@P@B@K+Q,np.inf)/max(1.,la.norm(P,np.inf))
    radii=np.abs(la.eigvals(A-B@K))
    H=np.zeros((5,11));H[np.arange(5),np.arange(5)]=1
    measurement_sigma=np.array([.001,.001,.002,.003,.003])
    process=np.diag([1e-8,1e-8,2e-7,2e-7,2e-7, 2e-6,2e-6,2e-5,2e-5,2e-5,5e-5])
    meas_cov=np.diag(measurement_sigma**2)
    Ppred=la.solve_discrete_are(A.T,H.T,process,meas_cov)
    L=la.solve(H@Ppred@H.T+meas_cov,H@Ppred).T
    observer_radius=float(max(abs(la.eigvals((np.eye(11)-L@H)@A))))
    phase_A,phase_B,_=linearize(qref,uref,phase=.713)
    phase_invariance=float(max(np.max(np.abs(phase_A-A)),np.max(np.abs(phase_B-B))))
    return {'schema':'wheelbot-profile/v1','model':'simplified single-side sagittal Upkie/sketch derivative; not a hardware digital twin',
      'asset':'assets/wheelbot/wheelbot.xml','assetSha256':hashlib.sha256(XML.read_bytes()).hexdigest(),'xmlSha256':hashlib.sha256(XML.read_bytes()).hexdigest(),
      'timestep':.002,'controlDt':DT,'substeps':5,'qOrder':['x','z','pitch','hip','knee','wheel'],'vOrder':['x_dot','z_dot','pitch_dot','hip_dot','knee_dot','wheel_dot'],
      'actuators':['hip_motor','knee_motor','wheel_motor'],'actuatorUnits':'N m','qref':qref.tolist(),'uref':uref.tolist(),
      'stateOrder':'[x,z,pitch,hip,knee,x_dot,z_dot,pitch_dot,hip_dot,knee_dot,wheel_dot] relative to qref/uref; wheel absolute phase omitted by axial rotational symmetry',
      'controlledIndices':CONTROLLED.tolist(),'measurementOrder':['x','z','pitch','hip','knee'],'measurementUnits':['m','m','rad','rad','rad'],
      'Qe':process.tolist(),'Re':meas_cov.tolist(),'kalmanPredictedCovariance':Ppred.tolist(),'measurementSigma':measurement_sigma.tolist(),'A':A.tolist(),'B':B.tolist(),'Q':Q.tolist(),'R':R.tolist(),'P':P.tolist(),'K':K.tolist(),'L':L.tolist(),
      'trimSolver':'scipy.optimize.least_squares/native qacc; fixed hip=.15,knee=-.3 rad', 'trimEvaluations':result.nfev,'trimQaccInf':trim,'dareNormalizedResidual':float(dare),'closedLoopRadius':float(max(radii)),
      'observerErrorRadius':observer_radius,'wheelPhaseInvarianceMaxAbs':phase_invariance,
      'designAvailable':bool(trim<1e-8 and dare<1e-8 and max(radii)<1 and observer_radius<1 and phase_invariance<1e-7),
      'modelMetadata':{'torsoMassKg':1.0,'eachLinkMassKg':.056,'wheelTireMassKg':.08715,'totalMassKg':float(m.body_mass.sum()),'linkLengthM':.25,'wheelRadiusM':.05,
       'derivedFrom':'pinned Upkie URDF torso, femur/tibia, tire values; motor assemblies omitted; primitive inertias recomputed analytically'},
      'limitsNm':[16.,16.,1.7],'solver':'scipy.linalg.solve_discrete_are; A/B from MuJoCo mjd_transitionFD and 5-step composition',
      'scope':'local design about solved bent-knee standing contact; no global or hardware stability claim'}

if __name__=='__main__':
    profile=design();OUT.write_text(json.dumps(profile,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:profile[k] for k in ['xmlSha256','trimQaccInf','dareNormalizedResidual','closedLoopRadius','observerErrorRadius','wheelPhaseInvarianceMaxAbs','designAvailable']},indent=2))
