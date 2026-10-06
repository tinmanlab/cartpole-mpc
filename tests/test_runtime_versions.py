"""Actual native binary identity; historical evidence cannot satisfy this gate."""
import hashlib, importlib.metadata, json, sys
from pathlib import Path
import mujoco
ROOT=Path(__file__).resolve().parents[1]
version=json.loads((ROOT/'package.json').read_text())['dependencies']['@mujoco/mujoco']
manifest=ROOT/'vendor/manifest.json'
report=dict(engineVersion=mujoco.mj_versionString(),engineNumber=mujoco.mj_version(),moduleVersion=mujoco.__version__,distributionVersion=importlib.metadata.version('mujoco'),interpreter=sys.executable,manifestSha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),expected=version)
print(json.dumps(report))
assert all(report[k]==version for k in ['engineVersion','moduleVersion','distributionVersion'])
assert json.loads(manifest.read_text())['packages']['@mujoco/mujoco']==version
assert 'mujoco=='+version in (ROOT/'requirements-native.txt').read_text().splitlines()
if '--require-latest' in sys.argv: assert version=='3.15.0', 'Latest engine upgrade is incomplete'
