"""Nonlinear motor-generated jump reference and scheduled linear feedback.

True state is used only for offline reference construction. Runtime TVLQR/KF
uses the planned reference plus the same five noisy position measurements.
No external force, mocap support, teleport or landing reset is permitted.
"""
from pathlib import Path
import hashlib
import json

import mujoco
import numpy as np
from scipy.linalg import solve
from scipy.optimize import least_squares

ROOT=Path(__file__).resolve().parents[1]
PROTOCOL=ROOT/'tests/fixtures/wheelbot_jump.json'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def make_context():
    c=json.loads(PROTOCOL.read_text())
    p=json.loads((ROOT/c['baselineProfile']).read_text())
    m=mujoco.MjModel.from_xml_path(str(ROOT/c['asset']))
    assert mujoco.__version__=='3.7.0' and m.nu==3 and m.nq==m.nv==6
    assert sha(ROOT/c['asset'])==p['assetSha256']
    assert m.opt.timestep==c['physicsDt']==.002 and c['controlDt']==.01
    return c,p,m


def run_design():
    c,p,m=make_context();d=mujoco.MjData(m);scratch=mujoco.MjData(m)
    q0=np.array(p['qref']);u0=np.array(p['uref']);x0=np.r_[q0,np.zeros(6)]
    idx=np.array(p['controlledIndices']);limits=np.array(c['actuatorLimitsNm'])
    cfg=c['referenceDesign'];N=round(c['durationSeconds']/c['controlDt']);substeps=5
    wheel=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'wheel_visual')
    mass=float(m.body_mass.sum())

    def set_data(data,x,u=None):
        mujoco.mj_resetData(m,data);data.qpos[:]=x[:6];data.qvel[:]=x[6:]
        if u is not None:data.ctrl[:]=u

    def step_with_contacts(x,u):
        set_data(d,x,u);contacts=[]
        for _ in range(substeps):
            mujoco.mj_step(m,d);contacts.append(int(d.ncon))
        return np.r_[d.qpos,d.qvel].copy(),contacts

    def step(x,u):
        return step_with_contacts(x,u)[0]

    def posture(t):
        t0=cfg['standSeconds'];tc=cfg['crouchSeconds'];te=cfg['extensionSeconds']
        if t<t0:return .15,0.,0.,'stand'
        if t<t0+tc:f=(t-t0)/tc;duration=tc;lo=.15;hi=cfg['crouchHipRad'];stage='crouch'
        elif t<t0+tc+te:f=(t-t0-tc)/te;duration=te;lo=cfg['crouchHipRad'];hi=.15;stage='thrust'
        else:return .15,0.,0.,'settle'
        s=10*f**3-15*f**4+6*f**5
        ds=(30*f*f-60*f**3+30*f**4)/duration
        dds=(60*f-180*f*f+120*f**3)/duration**2
        return lo+(hi-lo)*s,(hi-lo)*ds,(hi-lo)*dds,stage

    def design_command(x,t):
        h,hd,hdd,stage=posture(t)
        pitch=q0[2]+cfg['positionToPitch']*(x[0]+cfg['velocityWeightSeconds']*x[6])
        target=np.array([pitch,h,-2*h]);vel=np.array([0,hd,-2*hd]);acc=np.array([0,hdd,-2*hdd])
        desired=acc+cfg['jointKp']*(target-x[2:5])+cfg['jointKd']*(vel-x[8:11])
        set_data(scratch,x);mujoco.mj_forward(m,scratch);base=scratch.qacc[2:5].copy();mapping=np.zeros((3,3))
        for j in range(3):
            scratch.ctrl[:]=0;scratch.ctrl[j]=1;scratch.qacc_warmstart[:]=0;mujoco.mj_forward(m,scratch)
            mapping[:,j]=scratch.qacc[2:5]-base
        guess=np.linalg.lstsq(mapping,desired-base,rcond=1e-6)[0]
        def residual(u):
            scratch.ctrl[:]=u;scratch.qacc_warmstart[:]=0;mujoco.mj_forward(m,scratch)
            return scratch.qacc[2:5]-desired
        result=least_squares(residual,np.clip(guess,-limits+1e-8,limits-1e-8),bounds=(-limits,limits),max_nfev=12,ftol=1e-5,gtol=1e-5,xtol=1e-5)
        return result.x,stage

    def observation(x):
        set_data(d,x);mujoco.mj_forward(m,d);mujoco.mj_subtreeVel(m,d)
        clear=float(d.geom_xpos[wheel,2]-.05)
        contacts=sum(wheel in (d.contact[j].geom1,d.contact[j].geom2) for j in range(d.ncon))
        return {'com':d.subtree_com[1].tolist(),'wheelClearanceM':clear,'wheelContacts':contacts}

    xs=[x0.copy()];us=[];phases=[]
    for k in range(N):
        u,stage=design_command(xs[-1],k*.01);us.append(u);phases.append(stage);xs.append(step(xs[-1],u))
    obs=[observation(x) for x in xs]
    reference_metrics=metrics(c,p,xs,obs)
    if not reference_metrics['passed']:
        raise ValueError('Canonical-map jump reference rejected: '+json.dumps(reference_metrics))
    # Finite differences of the actual 10ms state map, not a fixed standing model.
    As=[];Bs=[];derivative_checks=[]
    for k in range(N):
        x=xs[k];u=us[k];A=np.zeros((11,11));B=np.zeros((11,3))
        for kind,count,output in [('x',11,A),('u',3,B)]:
            for j in range(count):
                estimates=[]
                for epsilon in c['derivativePolicy']['perturbations']:
                    dx=np.zeros(12);du=np.zeros(3)
                    if kind=='x':dx[idx[j]]=epsilon
                    else:du[j]=epsilon
                    plus,positive_contacts=step_with_contacts(x+dx,u+du)
                    minus,negative_contacts=step_with_contacts(x-dx,u-du)
                    estimates.append(((plus-minus)[idx]/(2*epsilon),positive_contacts==negative_contacts,epsilon))
                chosen=estimates[0] if estimates[0][1] else estimates[1]
                if not chosen[1] or not np.isfinite(chosen[0]).all():
                    raise ValueError(f'Jump derivative rejected at {k}/{kind}/{j}: contact branch disagreement')
                relative=float(np.max(np.abs(estimates[0][0]-estimates[1][0]))/max(1.,float(np.max(np.abs(estimates[1][0])))))
                derivative_checks.append({'sample':k,'kind':kind,'column':j,'epsilon':chosen[2],'relativeRefinementError':relative,'sameContactPattern':chosen[1]})
                output[:,j]=chosen[0]
        As.append(A);Bs.append(B)
    terminal=json.loads((ROOT/'assets/wheelbot/recovery_profile.json').read_text())
    Q=np.array(terminal['Q']);R=np.array(terminal['R'])
    P=np.array(terminal['P']);Ks=[None]*N
    for k in range(N-1,-1,-1):
        A,B=As[k],Bs[k];K=solve(R+B.T@P@B,B.T@P@A,assume_a='pos')
        F=A-B@K;P=Q+K.T@R@K+F.T@P@F;P=(P+P.T)*.5;Ks[k]=K
    # Time-varying covariance propagation is tied to the planned contact sequence.
    H=np.eye(11)[:5];Qe=np.array(p['Qe']);Re=np.array(p['Re']);I=np.eye(11)
    Ppred=np.array(p['kalmanPredictedCovariance']);L0=np.array(p['L'])
    P0=(I-L0@H)@Ppred@(I-L0@H).T+L0@Re@L0.T;Pe=P0.copy();Ls=[]
    for A in As:
        prior=A@Pe@A.T+Qe;S=H@prior@H.T+Re
        L=solve(S,H@prior,assume_a='pos').T
        Pe=(I-L@H)@prior@(I-L@H).T+L@Re@L.T;Pe=(Pe+Pe.T)*.5;Ls.append(L)
    profile={'schema':'wheelbot-jump-profile/v1','asset':c['asset'],'assetSha256':sha(ROOT/c['asset']),
             'baselineSha256':sha(ROOT/c['baselineProfile']),'recoverySha256':sha(ROOT/'assets/wheelbot/recovery_profile.json'),'protocolSha256':sha(PROTOCOL),
             'generatorSha256':sha(Path(__file__)),'controlDt':.01,'physicsDt':.002,'steps':N,
             'controlledIndices':idx.tolist(),'measurementIndices':[0,1,2,3,4],
             'measurementSigma':p['measurementSigma'],'limitsNm':limits.tolist(),
             'externalForce':[0,0,0],'ref':[x.tolist() for x in xs],'u':[u.tolist() for u in us],
             'phases':phases,'A':[A.tolist() for A in As],'B':[B.tolist() for B in Bs],
             'K':[K.tolist() for K in Ks],'L':[L.tolist() for L in Ls],
             'Q':Q.tolist(),'R':R.tolist(),'Qe':Qe.tolist(),'Re':Re.tolist(),'P0':P0.tolist(),
             'trajectoryCostSource':'Existing recovery Q/R and terminalP',
             'derivativeDiagnostics':{'method':'Two perturbation sizes and contact-sequence check','branchCrossings':0,'maxRelativeRefinementError':max(v['relativeRefinementError'] for v in derivative_checks),'largestGain':max(float(np.max(np.abs(k)))for k in Ks)},
             'target':c['target'],'envelope':c['envelope'],'referenceMetrics':reference_metrics,
             'method':'Native nonlinear reference + finite-horizon TVLQR and scheduled KF; NOT an online nonlinear optimizer',
             'scope':'The offline reference used true state; runtime feedback uses only estimated state and5measured positions. Scheduled contact transitions, no40Nrecovery or hardware guarantee.'}
    out=ROOT/'assets/wheelbot/jump_profile.json';out.write_text(json.dumps(profile,separators=(',',':'),allow_nan=False)+'\n')
    (ROOT/'test-results/jump_tracking_nominal.json').write_text(json.dumps({'profileSha256':sha(out),'observations':obs,'metrics':reference_metrics},indent=2)+'\n')
    (ROOT/'test-results/jump_final_derivative_checks.json').write_text(json.dumps(derivative_checks,indent=2)+'\n')
    print(json.dumps({'profileSha256':sha(out),'bytes':out.stat().st_size,'reference':reference_metrics},indent=2))
    return profile


def metrics(c,p,xs,obs):
    x=np.array(xs);co=np.array([o['com'] for o in obs]);apex=float(max(co[:,2]-co[0,2]));dt=c['controlDt']
    run=0;longest=0;takeoff=None;landing=None
    for i,o in enumerate(obs):
        air=o['wheelContacts']==0 and o['wheelClearanceM']>c['target']['wheelClearanceM']
        run=run+1 if air else 0;longest=max(longest,run)
        if air and takeoff is None:takeoff=i
        if takeoff is not None and i>takeoff and o['wheelContacts']>0 and landing is None:landing=i
    count=round(c['target']['settleSeconds']/dt);tail=x[-count:];qref=np.array(p['qref'])
    errors=np.max(np.abs(tail[:,:5]-qref[:5]),axis=0)
    envelope=bool(np.any(np.abs(x[:,0]-qref[0])>c['envelope']['positionMagnitudeM']) or np.any(np.abs(x[:,2]-qref[2])>c['envelope']['pitchErrorRad']) or np.any(x[:,1]<c['envelope']['minimumRootZM']))
    settle=bool(all(o['wheelContacts']>0 for o in obs[-count:]) and errors[0]<.015 and errors[2]<.025 and errors[3]<.03 and errors[4]<.03)
    return {'passed':bool(apex>=.01 and longest*dt>=.03 and settle and not envelope),
            'comApexM':apex,'longestFlightSeconds':longest*dt,'maxWheelClearanceM':max(o['wheelClearanceM'] for o in obs),
            'takeoffSample':takeoff,'landingSample':landing,'settled':settle,'envelopeFailure':envelope,
            'tailPositionM':float(errors[0]),'tailPitchRad':float(errors[2]),'tailHipRad':float(errors[3]),'tailKneeRad':float(errors[4])}


if __name__=='__main__':run_design()
