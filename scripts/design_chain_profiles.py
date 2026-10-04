"""Offline native linearization/DARE authority for the N-link experiment.
No handwritten dynamics or optimizer. Profiles are tied to exact MJCF hashes.
"""
from __future__ import annotations
import hashlib,json,sys
from pathlib import Path
import mujoco,numpy as np,scipy,scipy.linalg as la
ROOT=Path(__file__).resolve().parents[1]
DT=.02;HORIZON=30

def design(n):
    asset=ROOT/f'assets/chains/cartpole_{n}.xml';m=mujoco.MjModel.from_xml_path(str(asset));d=mujoco.MjData(m)
    dof=m.nv;nx=2*dof;A1=np.zeros((nx,nx));B1=np.zeros((nx,1))
    mujoco.mjd_transitionFD(m,d,1e-6,True,A1,B1,None,None)
    A=np.eye(nx);B=np.zeros((nx,1))
    for _ in range(4):B=A1@B+B1;A=A1@A
    # relative q -> absolute link angles; likewise for angular rates.
    W=np.eye(dof);W[1:,1:]=np.tril(np.ones((n,n)))
    T=la.block_diag(W,W);weights=np.r_[2,np.full(n,68.),.45,np.full(n,3.5)]
    Q=T.T@np.diag(weights)@T;R=.14
    eig=np.linalg.eigvals(A)
    def pbh(input_matrix,system):
        vals=[float(la.svdvals(np.hstack([complex(z)*np.eye(nx)-system,input_matrix]))[-1]) for z in np.linalg.eigvals(system)]
        return min(vals)
    H=np.hstack([np.eye(dof),np.zeros((dof,dof))]);oldH=H[:2]
    qc=np.diag(np.r_[np.full(dof,2e-7),np.full(dof,1e-5)]);rc=np.diag(np.full(dof,(2e-4)**2))
    out={'poles':n,'dof':dof,'nx':nx,'nu':1,'asset':f'assets/chains/cartpole_{n}.xml','assetSha256':hashlib.sha256(asset.read_bytes()).hexdigest(),
      'stateOrder':'[cart_x, relative_q_1..N, cart_v, relative_w_1..N]','measurementOrder':'[cart_x, relative_q_1..N]','controlDt':DT,'horizon':HORIZON,'forceLimit':10.,'railLimit':2.4,
      'A':A.tolist(),'B':B.tolist(),'Q':Q.tolist(),'R':R,'H':H.tolist(),'Qe':qc.tolist(),'Re':rc.tolist(),'angleMap':W.tolist(),
      'openLoopRadius':float(max(abs(eig))),'pbhControlSigmaMin':pbh(B,A),'pbhAllEncoderObservableSigmaMin':pbh(H.T,A.T),'pbhOldTwoSensorSigmaMin':pbh(oldH.T,A.T),
      'modelMetadata':{'cartMass':1.,'poleMassEach':.1,'poleFullLengthEach':1.,'totalMass':1+.1*n,'heightAbovePivot':float(n),'unactuatedHinges':n},'designAvailable':False}
    try:
        P=la.solve_discrete_are(A,B,Q,np.array([[R]]));K=la.solve(np.array([[R]])+B.T@P@B,B.T@P@A)
        residual=la.norm(A.T@P@A-P-A.T@P@B@K+Q,np.inf)/max(1,la.norm(P,np.inf))
        pred=la.solve_discrete_are(A.T,H.T,qc,rc);gain=la.solve(H@pred@H.T+rc,H@pred).T
        post=(np.eye(nx)-gain@H)@pred@(np.eye(nx)-gain@H).T+gain@rc@gain.T
        cr=float(max(abs(la.eigvals(A-B@K))));er=float(max(abs(la.eigvals((np.eye(nx)-gain@H)@A))))
        good=np.isfinite(P).all() and la.eigvalsh(P)[0]>0 and residual<=1e-9 and cr<1 and er<1
        out.update(P=P.tolist(),K=K.tolist(),kalmanGain=gain.tolist(),kalmanSteadyPosterior=post.tolist(),dareNormalizedResidual=float(residual),dareCondition=float(np.linalg.cond(P)),closedLoopRadius=cr,observerErrorRadius=er,designAvailable=bool(good))
        if not good:out['reason']='local Riccati/residual/stability numerical gate failed'
    except (la.LinAlgError,ValueError) as e:out['reason']=str(e)
    return out

if __name__=='__main__':
    profiles=[design(n) for n in range(1,9)]
    out={'schema':'cartpole-chain-designs/v1','physics':'MuJoCo 3.7.0 / Euler 4x5ms','versions':{'mujoco':mujoco.__version__,'scipy':scipy.__version__,'numpy':np.__version__},
      'sources':{'generator':'scripts/build_chain_assets.py','jacobians':'MuJoCo mjd_transitionFD + exact 4-substep composition at equilibrium','DARE':'scipy.linalg.solve_discrete_are'},
      'profiles':profiles,'scope':'local upright design, not swing-up or constrained global stability; stored covariance is a nominal steady-state design, not a calibrated runtime guarantee'}
    (ROOT/'assets/chains/profiles.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n')
    print(json.dumps([{k:p.get(k) for k in ['poles','designAvailable','closedLoopRadius','dareNormalizedResidual','dareCondition','pbhControlSigmaMin','pbhAllEncoderObservableSigmaMin','pbhOldTwoSensorSigmaMin','reason']} for p in profiles],indent=2))
