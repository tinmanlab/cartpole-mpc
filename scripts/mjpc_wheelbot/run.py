"""Generate task-only XML, assert plant identity, run sampler, evaluate outside Task."""
from pathlib import Path
import argparse, csv, hashlib, json, subprocess, sys, time, xml.etree.ElementTree as ET
import numpy as np
import mujoco
from build import ROOT, OUT, preflight, require_binary, digest
RUN_OUT=ROOT/'test-results/mjpc-admission'

def plant_snapshot(m):
    # Every numeric model array except task/sensor/name/buffer-address metadata.
    omit=('sensor_','numeric_','text_','tuple_','name_','names','signature','buffer')
    arrays={}
    for key in dir(m):
        if key.startswith('_') or key.startswith(omit): continue
        value=getattr(m,key)
        if isinstance(value,np.ndarray) and value.dtype.kind in 'biuf': arrays[key]=value.tolist()
    arrays['option']={key:np.asarray(getattr(m.opt,key)).tolist() for key in dir(m.opt) if not key.startswith('_') and isinstance(getattr(m.opt,key),(int,float,np.ndarray))}
    return arrays

def assert_plant_equal(base,augmented,numerics):
    a,b=plant_snapshot(base),plant_snapshot(augmented); allowed=[]
    if numerics=='reproducible':
        warmstart=int(mujoco.mjtDisableBit.mjDSBL_WARMSTART)
        if int(base.opt.disableflags)^int(augmented.opt.disableflags)!=warmstart: raise ValueError('reproducible profile must change only mjDSBL_WARMSTART')
        allowed=['disableflags: mjDSBL_WARMSTART']
        b['option']['disableflags']=a['option']['disableflags']
    elif numerics!='legacy': raise ValueError('unknown numerics profile')
    if a!=b: raise ValueError('plant changed: '+str([k for k in a if a[k]!=b[k]]))
    return allowed

VARIANTS={'baseline':(24,64,False),'safe_task':(24,64,True),'safe_compact':(8,128,True)}
NUMERICS={'legacy':0,'reproducible':1}
def model(variant='safe_task',numerics='reproducible'):
    if variant not in VARIANTS: raise ValueError('unknown task variant')
    if numerics not in NUMERICS: raise ValueError('unknown numerics profile')
    knots,rollouts,safe=VARIANTS[variant]
    original=ROOT/'assets/wheelbot/contact_model.xml'
    profile=json.loads((ROOT/'assets/wheelbot/contact_profile.json').read_text())
    if profile['xmlSha256']!=digest(original): raise ValueError('trim profile/model mismatch')
    tree=ET.parse(original); root=tree.getroot(); custom=ET.SubElement(root,'custom')
    base=mujoco.MjModel.from_xml_path(str(original))
    warmstart=int(mujoco.mjtDisableBit.mjDSBL_WARMSTART)
    settings={'safe_task':[int(safe)],'sampling_trajectories':[rollouts],'sampling_spline_points':[knots],'sampling_exploration':[.08,.3],'sampling_representation':[1],'trim_q':profile['qref'],'trim_u':profile['uref'],'standing_K':np.asarray(profile['K']).ravel().tolist(),'numerics_reproducible':[NUMERICS[numerics]],'numerics_base_disableflags':[int(base.opt.disableflags)],'numerics_base_enableflags':[int(base.opt.enableflags)]}
    for name,value in settings.items(): ET.SubElement(custom,'numeric',name=name,data=' '.join(map(str,value)))
    if numerics=='reproducible':
        if int(base.opt.disableflags)&warmstart: raise ValueError('base plant already disables mjDSBL_WARMSTART')
        option=root.find('option')
        if option is None: option=ET.SubElement(root,'option')
        flag=option.find('flag')
        if flag is None: flag=ET.SubElement(option,'flag')
        flag.set('warmstart','disable')
    sensors=ET.SubElement(root,'sensor')
    for name,dim,weight in [('orientation',2,15),('height',1,30),('pose',2,2),('movement',1,.2),('velocity',6,.015),('effort',3,.01),('joint_limit',2,100)]:
        ET.SubElement(sensors,'user',name=name,dim=str(dim),needstage='acc',user=f'0 {weight} 0 100')
    if safe: ET.SubElement(sensors,'user',name='penetration',dim='1',needstage='acc',user='0 100 0 100')
    RUN_OUT.mkdir(parents=True,exist_ok=True)
    target=RUN_OUT/('task-'+variant+'-'+numerics+'-'+str(time.time_ns())+'.xml'); tree.write(target,encoding='unicode')
    augmented=mujoco.MjModel.from_xml_path(str(target))
    a=plant_snapshot(base); allowed=assert_plant_equal(base,augmented,numerics)
    snapshot_hash=lambda value: hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()
    original_snapshot_sha256=snapshot_hash(a); runtime_snapshot_sha256=snapshot_hash(plant_snapshot(augmented))
    settled={}
    for side,sign in [('left',-1),('right',1)]:
        d=mujoco.MjData(augmented);d.qpos[:]=[0,.7,sign*1.1,.15,-.3,0]
        mujoco.mj_forward(augmented,d)
        initial_penetration=max([0]+[-float(c.dist) for c in d.contact])
        assert initial_penetration==0
        mujoco.mj_step(augmented,d,nstep=2000)
        settled[side]={'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'time':d.time,'contacts':d.ncon,'initial_penetration':initial_penetration}
    return target,profile,{'task_variant':variant,'numerics_profile':numerics,'residual_dimensions':int(augmented.nsensordata),'task_sensors':[{"name":augmented.sensor(i).name,"dimension":int(augmented.sensor_dim[i]),"norm_and_weight":augmented.sensor_user[i].tolist()} for i in range(augmented.nsensor)],'target_numerics':settings,'settled_both_sides':settled,'numeric_plant_sha256':original_snapshot_sha256,'original_snapshot_sha256':original_snapshot_sha256,'runtime_snapshot_sha256':runtime_snapshot_sha256,'numeric_fields_compared':len(a),'original_xml_sha256':digest(original),'runtime_xml_sha256':digest(target),'task_xml_sha256':digest(target),'allowed_flags_scope':allowed,'equality_scope':'all numeric model arrays and options; reproducible permits only disableflags XOR mjDSBL_WARMSTART','mechanical_model_equal':True,'plant_equal':True}

def evaluate(path,profile):
    with path.open() as f: rows=list(csv.DictReader(f))
    a={k:np.array([float(r[k]) for r in rows]) for k in rows[0] if k!='selectedpolicy'}
    elapsed=a['time']-a['time'][0]; tail=elapsed>=5-1e-8
    if not tail.any(): tail=elapsed>=max(0,elapsed[-1]-1)
    q=profile['qref']; pitch=np.arctan2(np.sin(a['q2']-q[2]),np.cos(a['q2']-q[2]))
    metrics={'duration_s':float(elapsed[-1]),'final_root_height_m':float(a['q1'][-1]),'max_root_height_m':float(a['q1'].max()),'tail_min_height_m':float(a['q1'][tail].min()),'tail_max_pitch_error_rad':float(abs(pitch[tail]).max()),'tail_max_joint_error_rad':max(float(abs(a[f'q{i}'][tail]-q[i]).max()) for i in [3,4]),'tail_max_speed':max(float(abs(a[f'v{i}'][tail]).max()) for i in range(6)),'max_penetration_m':float(a['penetration'].max()),'max_joint_excursion_rad':float(a['limit'].max()),'max_actual_torques_nm':[float(abs(a[f'force{i}']).max()) for i in range(3)],'max_controls_nm':[float(abs(a[f'u{i}']).max()) for i in range(3)],'initial_qpos':[float(a[f'q{i}'][0]) for i in range(6)],'initial_qvel':[float(a[f'v{i}'][0]) for i in range(6)],'contact_count_range':[int(a['contacts'].min()),int(a['contacts'].max())],'finite':all(np.isfinite(v).all() for v in a.values())}
    gates={'complete_6s':len(rows)==3001 and abs(metrics['duration_s']-6)<1e-7 and bool(np.allclose(np.diff(a['time']),.002,rtol=0,atol=1e-10)),'height':metrics['tail_min_height_m']>.9*q[1],'pitch':metrics['tail_max_pitch_error_rad']<.08,'joints':metrics['tail_max_joint_error_rad']<.12,'speed':metrics['tail_max_speed']<.3,'penetration':metrics['max_penetration_m']<.02,'limits':metrics['max_joint_excursion_rad']<.05,'torques':all(x<=y+1e-10 for x,y in zip(metrics['max_actual_torques_nm'],[16,16,1.7])),'finite':metrics['finite']}
    return {'metrics':metrics,'gates':gates,'getup_success':all(gates.values())}

def run(mode,seconds,staged,variant='safe_task',numerics='reproducible'):
    if mode not in ['standing','perturbed','perturbed-left','left','right'] or not np.isfinite(seconds) or seconds<=0 or seconds>6 or abs(seconds/.002-round(seconds/.002))>1e-8 or staged not in [0,1]: raise ValueError('invalid scenario/duration/staging')
    OUT.mkdir(parents=True,exist_ok=True); require_binary(OUT/'wheelbot')
    builds=[p for p in sorted(OUT.glob('build-*.json')) if json.loads(p.read_text()).get('success')]
    build_path=builds[-1]; build=json.loads(build_path.read_text())
    if build['binary_sha256']!=digest(OUT/'wheelbot') or build['adapter_sha256']!=digest(ROOT/'scripts/mjpc_wheelbot/main.cc'): raise ValueError('stale binary/source binding')
    env=build.get('environment',{}); source=preflight(**{k:Path(v) for k,v in env.items() if k in ['source','mujoco']})
    target,profile,fingerprint=model(variant,numerics); ident=f'{variant}-{numerics}-{mode}-{staged}-{time.time_ns()}'
    RUN_OUT.mkdir(parents=True,exist_ok=True)
    trace=RUN_OUT/(ident+'.csv'); cmd=[str(OUT/'wheelbot'),str(target),str(trace),mode,str(seconds),str(staged),numerics]
    start=time.monotonic()
    try:
        result=subprocess.run(cmd,capture_output=True,text=True,timeout=300)
        rc=result.returncode; stdout=result.stdout; stderr=result.stderr
    except subprocess.TimeoutExpired as e:
        rc=124;stdout=(e.stdout or b'').decode() if isinstance(e.stdout,bytes) else e.stdout or '';stderr='300 second verification time cap'
    receipt={'schema':'wheelbot-mjpc-adoption/v1','id':ident,'command':cmd,'returncode':rc,'wall_seconds':time.monotonic()-start,'source':source,'plant':fingerprint['numeric_plant_sha256'],'task_model':fingerprint,'task_xml':str(target.relative_to(ROOT)),'numerics_profile':numerics,'trace':str(trace.relative_to(ROOT)),'mode':mode,'variant':variant,'build_receipt':str(build_path.relative_to(ROOT)),'staged':bool(staged),'observer_integration':False,'privileged_full_state':True,'external_forces':False,'physics_dt_s':.002,'replan_dt_s':.01,'action_mapping':'official linear spline or explicit existing LQR feedback recomputed and held each 2ms; source in CSV','initialization':('standing trim + .02rad pitch' if mode=='perturbed' else 'standing trim - .02rad pitch' if mode=='perturbed-left' else 'standing trim') if mode in ['standing','perturbed','perturbed-left'] else f'{mode}: q=[0,.7,{(-1 if mode=="left" else 1)*1.1},.15,-.3,0], v=0, ctrl=0, 2000 uninterrupted gravity steps','stderr':stderr,'binary_sha256':digest(OUT/'wheelbot'),'adapter_sha256':digest(ROOT/'scripts/mjpc_wheelbot/main.cc'),'trim_q':profile['qref'],'profile_sha256':digest(ROOT/'assets/wheelbot/contact_profile.json')}
    try: receipt['planner']=json.loads(stdout)
    except (ValueError,TypeError): receipt['stdout']=stdout
    if rc==0:
        receipt.update(evaluate(trace,profile))
    else:
        receipt['getup_success']=False
        if trace.exists() and trace.stat().st_size:
            try: receipt['partial_trajectory']=evaluate(trace,profile)
            except (ValueError,KeyError): pass
    receipt['local_standing_success']=bool(mode in ['standing','perturbed','perturbed-left'] and receipt.get('getup_success'))
    if mode in ['standing','perturbed','perturbed-left']: receipt['getup_success']=False
    receipt['trace_sha256']=digest(trace) if trace.exists() else None
    receipt['raw_artifacts']={}
    for kind in ['initial','predictions']:
        artifact=Path(str(trace)+'.'+kind+'.json')
        receipt['raw_artifacts'][kind]={'path':str(artifact.relative_to(ROOT)),'sha256':digest(artifact) if artifact.exists() else None}
    path=RUN_OUT/(ident+'.json');path.write_text(json.dumps(receipt,indent=2)+'\n')
    current_path=ROOT/'evidence/wheelbot_mjpc_current.json'
    current=json.loads(current_path.read_text()) if current_path.exists() else {'schema':'wheelbot-mjpc-current/v2','runs':[],'historical_receipts':[str(p.relative_to(ROOT)) for p in sorted((ROOT/'evidence').glob('wheelbot_mjpc_*.json'))]}
    independent=ROOT/'test-results/mjpc-independent-replay.json'
    if 'transfer' not in current: current['transfer']={'native_wasm_closed_loop_verified':False,'observer_integration':False,'independent_replay_path':str(independent.relative_to(ROOT)),'independent_replay_sha256':digest(independent),'independent_replay_results':json.loads(independent.read_text())['results'],'coordinator_native_report':{'one_step_max_error':1.04e-8,'full_open_loop_error_approximately':35,'divergence_after_sample_approximately':490},'interpretation':'Native same-history replay and first nominal plan agreement do not establish WASM closed-loop transfer.'}
    current['runs'].append({'receipt':str(path.relative_to(ROOT)),'receipt_sha256':digest(path),'trace_sha256':digest(trace) if trace.exists() else None,**{k:receipt[k] for k in ['variant','mode','returncode','adapter_sha256','binary_sha256']},**{k:receipt[k] for k in ['planner','metrics','gates','getup_success','local_standing_success'] if k in receipt}})
    current_path.write_text(json.dumps(current,indent=2)+'\n')
    print(path);print(json.dumps({k:v for k,v in receipt.items() if k in ['returncode','planner','metrics','gates','getup_success','stderr','wall_seconds']}),flush=True)
    return rc
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario',choices=['standing','perturbed','perturbed-left','left','right'])
    parser.add_argument('seconds',type=float)
    parser.add_argument('staged',type=int,nargs='?',default=0,choices=[0,1])
    parser.add_argument('--variant',choices=VARIANTS,default='safe_task')
    parser.add_argument('--numerics',choices=NUMERICS,default='reproducible')
    args=parser.parse_args()
    sys.exit(run(args.scenario,args.seconds,args.staged,args.variant,args.numerics))
