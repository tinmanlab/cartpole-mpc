"""Package pinned, unmodified upstream runtimes for GitHub Pages (no CDN).
The tiny CommonJS wrapper only packages quadprog; no numerical code is changed.
"""
from pathlib import Path
import argparse, hashlib, json, shutil
ROOT=Path(__file__).resolve().parents[1]

def artifacts():
    files={}
    versions=json.loads((ROOT/'package.json').read_text())['dependencies']
    version=versions['@mujoco/mujoco']
    mj=ROOT/'node_modules/@mujoco/mujoco';qp=ROOT/'node_modules/quadprog'
    assert json.loads((mj/'package.json').read_text())['version']==version
    assert json.loads((qp/'package.json').read_text())['version']==versions['quadprog']
    for name in ['mujoco.js','mujoco.wasm','README.md']:
        files['vendor/mujoco/'+name]=(mj/name).read_bytes()
    files['vendor/mujoco/package.json']=(json.dumps({'type':'module','version':version},separators=(',',':'))+'\n').encode()
    files['vendor/mujoco/version.mjs']=('export const MUJOCO_VERSION='+json.dumps(version)+';\n').encode()
    files['vendor/mujoco/LICENSE']=(ROOT/'licenses/Apache-2.0.txt').read_bytes()
    files['vendor/quadprog/LICENSE']=(qp/'LICENSE').read_bytes()
    files['vendor/quadprog/README.md']=(qp/'README.md').read_bytes()
    sources=[qp/'index.js',*sorted((qp/'lib').glob('*.js'))]
    text="// quadprog@1.6.1, MIT. Upstream sources are unchanged inside module wrappers.\n(function(){const factories={\n"
    for p in sources:
        name=p.relative_to(qp).with_suffix('').as_posix()
        text+=json.dumps(name)+':function(module,exports,require){\n'+p.read_text()+'\n},\n'
    text+='''};const cache={};function load(id){id=id.replace(/\\.js$/,'');if(cache[id])return cache[id].exports;
const module={exports:{}};cache[id]=module;factories[id](module,module.exports,name=>load(name.startsWith('./')?(id.includes('/')?id.slice(0,id.lastIndexOf('/')+1):'')+name.slice(2):name));return module.exports;}
globalThis.Quadprog=load('index');})();\n'''
    files['vendor/quadprog/quadprog.js']=text.encode()
    manifest={'schema':'cartpole-runtime-assets/v1','packages':{'@mujoco/mujoco':version,'quadprog':versions['quadprog']},
              'origins':{'@mujoco/mujoco':f'https://github.com/google-deepmind/mujoco/tree/{version}/wasm','quadprog':'https://github.com/albertosantini/quadprog'},
              'files':{p:{'sha256':hashlib.sha256(b).hexdigest(),'bytes':len(b)} for p,b in files.items()},
              'quadprog_sources':{p.relative_to(qp).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}
    files['vendor/manifest.json']=(json.dumps(manifest,indent=2)+'\n').encode()
    return files

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--check',action='store_true');args=ap.parse_args()
    for path,data in artifacts().items():
        p=ROOT/path
        if args.check:
            assert p.exists() and p.read_bytes()==data,'Runtime asset differs from pinned package: '+path
        else:
            p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(data)
    print('pinned runtime assets: '+('verified' if args.check else 'packaged'))
