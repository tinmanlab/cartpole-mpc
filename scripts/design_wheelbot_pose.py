"""Reproducible native MuJoCo posture atlas; never rewrites the model."""
import hashlib, json, sys
from pathlib import Path
import numpy as np
import mujoco, scipy
import scipy.linalg as la
from scipy.optimize import least_squares
from design_wheelbot_live import linearize, atomic_json, XML, ROOT

HASH='17fec38c99f4ba42c99052570e8951b35cb4d041543ef47761baff91c4a50382'

def build():
    assert hashlib.sha256(XML.read_bytes()).hexdigest()==HASH
    m=mujoco.MjModel.from_xml_path(str(XML)); d=mujoco.MjData(m)
    profiles=[]
    base=json.loads((ROOT/'assets/wheelbot/live_profile.json').read_text())
    for z in np.linspace(.36,.49,5):
        for pitch in np.linspace(-np.pi/18,np.pi/18,5):
            bend=np.arccos((z+.00025-.05)/.5)
            def residual(v):
                mujoco.mj_resetData(m,d)
                d.qpos[:]=[0,z,pitch,v[0],v[1],0];d.ctrl[:]=v[2:]
                mujoco.mj_forward(m,d)
                return d.qacc.copy()
            def geometry(v):
                d.qpos[:]=[0,z,pitch,*v,0];mujoco.mj_forward(m,d)
                return [d.xpos[4,2]-.0498,d.subtree_com[1,0]-d.xpos[4,0]]
            seed=least_squares(geometry,[bend-pitch,-2*bend],gtol=1e-13).x
            fit=least_squares(residual,[*seed,0,1,0],diff_step=1e-5,xtol=1e-13,ftol=1e-13,gtol=1e-13,max_nfev=1000)
            error=float(np.max(np.abs(residual(fit.x))))
            if error>1e-7 or abs(fit.x[0])>1.24 or abs(fit.x[1])>2.49:
                raise ValueError((z,pitch,error,fit.x.tolist()))
            qref=d.qpos.copy();uref=d.ctrl.copy()
            A,B,_=linearize(qref,uref)
            Q=np.array(base['Q']);R=np.array(base['R']);H=np.array(base['measurementModel'])
            P=la.solve_discrete_are(A,B,Q,R);K=la.solve(R+B.T@P@B,B.T@P@A)
            Pe=la.solve_discrete_are(A.T,H.T,np.array(base['Qe']),np.array(base['Re']))
            L=la.solve(H@Pe@H.T+np.array(base['Re']),H@Pe).T
            p={**base,'qref':qref.tolist(),'uref':uref.tolist(),'trimQaccInf':error,
               'trimSolver':'Native qacc with fixed height/pitch; geometric COM-over-wheel initialization',
               'closedLoopRadius':float(max(abs(la.eigvals(A-B@K)))),
               'observerErrorRadius':float(max(abs(la.eigvals((np.eye(11)-L@H)@A)))),
               'dareNormalizedResidual':float(la.norm(A.T@P@A-P-A.T@P@B@K+Q,np.inf)/max(1,la.norm(P,np.inf))),
               **{name:value.tolist() for name,value in [('A',A),('B',B),('K',K),('L',L),('P',P),('kalmanPredictedCovariance',Pe)]}}
            phaseA,phaseB,_=linearize(qref,uref,.713)
            p['wheelPhaseInvarianceMaxAbs']=float(max(np.max(abs(phaseA-A)),np.max(abs(phaseB-B))))
            p['designAvailable']=error<1e-7 and p['closedLoopRadius']<1 and p['observerErrorRadius']<1
            assert abs(p['qref'][1]-z)<1e-7 and abs(p['qref'][2]-pitch)<1e-7
            p['trimEvaluations']=fit.nfev
            p['modelMetadata']={**base['modelMetadata'],'postureAnglesDeg':{'hip':float(np.degrees(qref[3])),'knee':float(np.degrees(qref[4])),'kneeFlexion':float(abs(np.degrees(qref[4])))}}
            p['target']={'z':float(z),'pitch':float(pitch)}
            profiles.append(p)
    bundle={'schema':'wheelbot-pose-atlas/v1','assetSha256':HASH,'controlDt':.01,
      'ranges':{'x':[-1,1],'z':[.36,.49],'pitch':[-float(np.pi/18),float(np.pi/18)]},
      'referenceLimits':{'velocity':[.12,.015,.04],'acceleration':[.12,.03,.08]},
      'capabilities':{'jump':False,'recover':False},'profiles':profiles,
      'versions':{'mujoco':mujoco.__version__,'scipy':scipy.__version__,'numpy':np.__version__},
      'scope':'Native solved static trims; bilinear scheduled LQR/KF is evaluated empirically, no time-varying Lyapunov claim.'}
    atomic_json(ROOT/'assets/wheelbot/pose_profiles.json',bundle)
    print(json.dumps({'count':len(profiles),'maxTrim':max(p['trimQaccInf'] for p in profiles),'maxPole':max(p['closedLoopRadius'] for p in profiles),'versions':bundle['versions']}))

if __name__=='__main__': build()
