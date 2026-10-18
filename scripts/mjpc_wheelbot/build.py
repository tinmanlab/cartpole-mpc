"""Build unchanged, pinned MJPC sources against the explicitly selected MuJoCo."""
from pathlib import Path
import argparse, concurrent.futures, ctypes, hashlib, json, re, subprocess, sys, time
ROOT=Path(__file__).resolve().parents[2]
SOURCE=Path('/home/tinman/repos/mujoco_mpc')
MUJOCO=Path('/home/tinman/cartpole-mpc-audit-20261004/.venv-native/lib/python3.10/site-packages/mujoco')
OLD=SOURCE/'build_cream_official'
PIN='ff572a21e7c2bf9fda62e1862a758da7e9a8719b'
FILES=['task.cc','norm.cc','utilities.cc','trajectory.cc','states/state.cc','spline/spline.cc','threadpool.cc','planners/planner.cc','planners/sampling/planner.cc','planners/sampling/policy.cc']
OUT=ROOT/'test-results/mjpc-adopt'
def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def preflight(source=SOURCE,mujoco=MUJOCO):
    head=subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()
    if head!=PIN: raise ValueError('wrong upstream source pin')
    if subprocess.check_output(['git','-C',str(source),'diff','HEAD','--']): raise ValueError('modified upstream tracked source')
    header=mujoco/'include/mujoco/mujoco.h'
    version=int(re.search(r'#define mjVERSION_HEADER (\d+)',header.read_text())[1])
    if version!=3007000: raise ValueError('wrong header version')
    lib=mujoco/'libmujoco.so.3.7.0'
    if ctypes.CDLL(str(lib)).mj_version()!=version: raise ValueError('wrong runtime version')
    return {'pin':head,'tracked_source_clean':True,'header_version':version,'runtime_version':version,'library_sha256':digest(lib),'source_sha256':{f:digest(source/'mjpc'/f) for f in FILES}}
def require_binary(path):
    if not path.is_file(): raise ValueError('missing binary; build first')
    subprocess.run([str(path),'--version'],check=True,capture_output=True,text=True)
def build(source=SOURCE,mujoco=MUJOCO,old=OLD):
    OUT.mkdir(parents=True,exist_ok=True)
    receipt=preflight(source,mujoco); receipt['environment']={'source':str(source),'mujoco':str(mujoco),'abseil_build':str(old)}; receipt['commands']=[]
    flags=['g++','-std=c++17','-O2','-pthread','-ffunction-sections','-fdata-sections','-I'+str(source),'-I'+str(mujoco/'include'),'-I'+str(old/'_deps/abseil-cpp-src')]
    if (ROOT/'scripts/mjpc_wheelbot/compat.h').exists(): flags+=['-include',str(ROOT/'scripts/mjpc_wheelbot/compat.h')]
    sources=[source/'mjpc'/f for f in FILES]+[ROOT/'scripts/mjpc_wheelbot/main.cc']
    def compile_one(p):
        obj=OUT/(p.parent.name+'_'+p.stem+'.o'); cmd=flags+['-c',str(p),'-o',str(obj)]
        r=subprocess.run(cmd,capture_output=True,text=True); return obj,{'command':cmd,'returncode':r.returncode,'stderr':r.stderr}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool: results=list(pool.map(compile_one,sources))
    receipt['commands']=[r for _,r in results]
    if all(r['returncode']==0 for _,r in results):
        cmd=['clang++-13','-flto','-pthread','-Wl,--gc-sections',*[str(o) for o,_ in results],str(mujoco/'libmujoco.so.3.7.0'),'-Wl,-rpath,'+str(mujoco),'-Wl,--start-group',*[str(p) for p in sorted((old/'lib').glob('libabsl*.a'))],'-Wl,--end-group','-ldl','-o',str(OUT/'wheelbot')]
        r=subprocess.run(cmd,capture_output=True,text=True); receipt['commands'].append({'command':cmd,'returncode':r.returncode,'stderr':r.stderr})
    receipt['success']=len(receipt['commands'])==12 and all(r['returncode']==0 for r in receipt['commands'])
    receipt['compatibility_shim']=(ROOT/'scripts/mjpc_wheelbot/compat.h').read_text() if (ROOT/'scripts/mjpc_wheelbot/compat.h').exists() else None
    receipt['adapter_sha256']=digest(ROOT/'scripts/mjpc_wheelbot/main.cc')
    receipt['upstream_rechecked']=preflight(source,mujoco)['tracked_source_clean']
    if receipt['success']:
        require_binary(OUT/'wheelbot'); receipt['binary_sha256']=digest(OUT/'wheelbot')
        receipt['adapter_files_sha256']={p.name:digest(p) for p in (ROOT/'scripts/mjpc_wheelbot').glob('*') if p.is_file()}
        (OUT/('main-'+receipt['adapter_sha256']+'.cc')).write_bytes((ROOT/'scripts/mjpc_wheelbot/main.cc').read_bytes())
    path=OUT/('build-'+str(time.time_ns())+'.json');path.write_text(json.dumps(receipt,indent=2)+'\n')
    print(path)
    for r in receipt['commands']:
        if r['returncode']: print(r['stderr'])
    return 0 if receipt['success'] else 1
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=SOURCE)
    parser.add_argument('--mujoco',type=Path,default=MUJOCO)
    parser.add_argument('--abseil-build',type=Path,default=OLD)
    args=parser.parse_args()
    sys.exit(build(args.source,args.mujoco,args.abseil_build))
