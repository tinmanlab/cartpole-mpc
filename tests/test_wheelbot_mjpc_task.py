"""Native task representation, plant identity and independent acceptance gates."""
import csv,hashlib,json,subprocess,sys,tempfile,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/mjpc_wheelbot'))
import run
out=ROOT/'test-results/mjpc-adopt'
profile=None
for variant,(knots,rollouts,safe) in run.VARIANTS.items():
    path,profile,fp=run.model(variant)
    assert fp['plant_equal'] and fp['mechanical_model_equal']
    assert fp['equality_scope']=='all numeric model arrays and options; reproducible permits only disableflags XOR mjDSBL_WARMSTART'
    m=mujoco.MjModel.from_xml_path(str(path))
    assert m.nsensordata==(18 if safe else 17)
    assert m.sensor('joint_limit').user[1]==100
    if safe: assert m.sensor('penetration').user[1]==100
    assert m.numeric('sampling_spline_points').data[0]==knots
    assert m.numeric('sampling_trajectories').data[0]==rollouts
    assert m.numeric('numerics_reproducible').data[0]==1
    assert m.numeric('numerics_base_disableflags').data[0]==0
    assert m.numeric('numerics_base_enableflags').data[0]==0
    assert fp['plant_equal'] and fp['allowed_flags_scope']==['disableflags: mjDSBL_WARMSTART']
    assert fp['original_snapshot_sha256'] and fp['runtime_snapshot_sha256']
    assert fp['original_xml_sha256'] and fp['runtime_xml_sha256']==fp['task_xml_sha256']
    assert m.opt.disableflags==int(mujoco.mjtDisableBit.mjDSBL_WARMSTART)
    np.testing.assert_array_equal(m.numeric('trim_q').data,profile['qref'])
    np.testing.assert_array_equal(m.numeric('trim_u').data,profile['uref'])
    result=subprocess.run([str(out/'wheelbot'),str(path),str(out/'unused.csv'),'check-task','.002','0','reproducible'],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    r=json.loads(result.stdout)
    np.testing.assert_allclose(r['joint_residuals'],[2,1] if safe else [.02,.01])
    assert r['penetration_residual']==(1 if safe else 0)
    # Deliberate numeric mutation must be detected by the physical comparison.
    baseline=run.plant_snapshot(m);m.body_mass[1]+=.01
    assert baseline!=run.plant_snapshot(m)
    try: run.assert_plant_equal(mujoco.MjModel.from_xml_path(str(ROOT/'assets/wheelbot/contact_model.xml')),m,'legacy')
    except ValueError: pass
    else: raise AssertionError('mass change escaped plant comparison')
legacy_path,_,legacy=run.model('safe_task','legacy')
legacy_model=mujoco.MjModel.from_xml_path(str(legacy_path))
legacy_base=mujoco.MjModel.from_xml_path(str(ROOT/'assets/wheelbot/contact_model.xml'))
assert legacy['plant_equal'] and legacy['allowed_flags_scope']==[]
assert legacy_model.opt.disableflags==legacy_base.opt.disableflags
assert legacy_model.numeric('numerics_reproducible').data[0]==0
try:
    run.model('safe_task','guess')
except ValueError:
    pass
else:
    raise AssertionError('unknown numerics profile accepted')
assert legacy_path!=run.model('safe_task','legacy')[0]
source=ROOT/'assets/wheelbot/contact_model.xml'
source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
with tempfile.TemporaryDirectory(dir=out) as temp:
    temp=Path(temp)
    for name,mutate in [
        ('warmstart',lambda root: root.find('./option/flag').set('warmstart','enable')),
        ('contact',lambda root: root.find('./option/flag').set('contact','disable')),
        ('timestep',lambda root: root.find('./option').set('timestep','.004')),
    ]:
        tree=ET.parse(path);mutate(tree.getroot());bad=temp/(name+'.xml');tree.write(bad,encoding='unicode')
        result=subprocess.run([str(out/'wheelbot'),str(bad),str(temp/'unused.csv'),'check-task','.002','0','reproducible'],capture_output=True,text=True)
        assert result.returncode==2,(name,result.returncode,result.stderr)
    result=subprocess.run([str(out/'wheelbot'),str(path),str(temp/'unused.csv'),'check-task','.002','0','unknown'],capture_output=True,text=True)
    assert result.returncode==2,('unknown numerics',result.returncode,result.stderr)
assert hashlib.sha256(source.read_bytes()).hexdigest()==source_hash
for scenario,duration,stage in [('right','nan','0'),('right','inf','0'),('wrong','6','0'),('right','6','2'),('right','6junk','0'),('right','.003','0')]:
    r=subprocess.run([str(out/'wheelbot'),str(path),str(out/'unused.csv'),scenario,duration,stage,'reproducible'],capture_output=True,text=True)
    assert r.returncode==2,(scenario,duration,stage)
# A perfect trim trajectory passes; missing data or any threshold violation fails.
keys=['time']+[f'{prefix}{i}' for prefix in ['q','v'] for i in range(6)]+[f'{prefix}{i}' for prefix in ['u','force'] for i in range(3)]+['contacts','penetration','limit','cost','iteration']
rows=[]
for k in range(3001):
    row={key:0. for key in keys};row['time']=k*.002
    row.update({f'q{i}':q for i,q in enumerate(profile['qref'])});rows.append(row)
with tempfile.TemporaryDirectory(dir=out) as temp:
    p=Path(temp)/'trace.csv'
    def evaluate(data):
        with p.open('w') as f:
            writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows(data)
        return run.evaluate(p,profile)
    assert evaluate(rows)['getup_success']
    assert not evaluate(rows[::2])['getup_success']
    assert not evaluate(rows[:26])['getup_success']
    for field,value in [('q1',.1),('q2',.2),('q3',.4),('v5',.31),('force2',1.71),('penetration',.021),('limit',.051),('cost',float('nan'))]:
        old=rows[-1][field];rows[-1][field]=value
        assert not evaluate(rows)['getup_success'],field
        rows[-1][field]=old
print('PASS: named 17/18 dimensions, exact residual scales, weights, targets, plant fingerprint, CLI validation, complete-six-second false-positive gates')
