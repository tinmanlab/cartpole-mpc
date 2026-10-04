"""Independent NumPy KF correction/Joseph covariance on the actual nonlinear prediction."""
from pathlib import Path
import json,numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1]
f=json.loads((ROOT/'test-results/chain_ekf_fixture.json').read_text());out=[]
for c in f['rows']:
    p=c['profile'];F,H,Q,R=[np.asarray(v) for v in [c['F'],p['H'],p['Qe'],p['Re']]]
    model=mujoco.MjModel.from_xml_path(str(ROOT/p['asset']));data=mujoco.MjData(model);before=np.array(c['before']['x'])
    data.qpos[:]=before[:p['dof']];data.qvel[:]=before[p['dof']:];data.ctrl[0]=.15;nativeF=np.eye(p['nx'])
    for _ in range(4):
        A1=np.zeros_like(nativeF);mujoco.mjd_transitionFD(model,data,1e-6,True,A1,None,None,None);nativeF=A1@nativeF;mujoco.mj_step(model,data)
    xp=np.r_[data.qpos,data.qvel];xp[1:p['dof']]=np.arctan2(np.sin(xp[1:p['dof']]),np.cos(xp[1:p['dof']]))
    native_prediction_error=float(np.max(np.abs(xp-c['prediction'])));native_jacobian_error=float(np.max(np.abs(nativeF-F)))
    assert native_prediction_error<=1e-10 and native_jacobian_error<=1e-7
    Pm=F@np.asarray(c['before']['P'])@F.T+Q;xm=np.asarray(c['prediction']);y=np.asarray(c['measurement'])
    S=H@Pm@H.T+R;K=np.linalg.solve(S,H@Pm).T;innovation=y-H@xm
    innovation[1:]=np.arctan2(np.sin(innovation[1:]),np.cos(innovation[1:]));x=xm+K@innovation
    x[1:p['dof']]=np.arctan2(np.sin(x[1:p['dof']]),np.cos(x[1:p['dof']]))
    I=np.eye(p['nx'])-K@H;P=I@Pm@I.T+K@R@K.T;nis=float(innovation@np.linalg.solve(S,innovation))
    xe=float(np.max(np.abs(x-c['after']['x'])));pe=float(np.max(np.abs(P-c['after']['P'])))
    se=float(np.max(np.abs(S-c['after']['last']['S'])));ne=abs(nis-c['after']['last']['nis'])
    assert max(xe,pe,se,ne)<=1e-8;assert np.linalg.eigvalsh(P).min()>0
    out.append({'poles':p['poles'],'nativePredictionError':native_prediction_error,'nativeJacobianError':native_jacobian_error,'meanMaxError':xe,'covarianceMaxError':pe,'innovationCovarianceMaxError':se,'nisError':ne,'posteriorMinEigenvalue':float(np.linalg.eigvalsh(P).min()),'passed':True})
report={'schema':'cartpole-chain-ekf-reference/v1','rows':out,'passed':True,'scope':'Numerical update correctness, not calibrated real-world consistency or dominance over stationary KF'}
(ROOT/'evidence/chain_ekf_reference.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(out,indent=2))
