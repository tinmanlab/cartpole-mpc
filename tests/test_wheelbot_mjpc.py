"""Small fail-closed preflight check; no native planner execution needed."""
import importlib.util
from pathlib import Path
import tempfile, unittest.mock as mock
spec=importlib.util.spec_from_file_location('build',Path(__file__).resolve().parents[1]/'scripts/mjpc_wheelbot/build.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
def rejects(call, fragment):
    try: call()
    except ValueError as e: assert fragment in str(e),str(e)
    else: raise AssertionError('invalid input accepted')
with tempfile.TemporaryDirectory() as t:
    rejects(lambda:b.require_binary(Path(t)/'absent'),'missing binary')
    with mock.patch.object(b.subprocess,'check_output',return_value='wrong\n'):
        rejects(b.preflight,'wrong upstream')
    with mock.patch.object(b.subprocess,'check_output',side_effect=[b.PIN+'\n',b'dirty']):
        rejects(b.preflight,'modified upstream')
    p=Path(t)/'include/mujoco';p.mkdir(parents=True);(p/'mujoco.h').write_text('#define mjVERSION_HEADER 323\n')
    with mock.patch.object(b.subprocess,'check_output',side_effect=[b.PIN+'\n',b'']):
        rejects(lambda:b.preflight(mujoco=Path(t)),'wrong header')
print('PASS: missing binary, wrong pin, dirty source, wrong version rejected')
with tempfile.TemporaryDirectory() as t:
    p=Path(t)/'include/mujoco';p.mkdir(parents=True)
    (p/'mujoco.h').write_text('#define mjVERSION_HEADER 3007000\n')
    with mock.patch.object(b.subprocess,'check_output',side_effect=[b.PIN+'\n',b'']), mock.patch.object(b.ctypes,'CDLL') as lib:
        lib.return_value.mj_version.return_value=323
        rejects(lambda:b.preflight(mujoco=Path(t)),'wrong runtime')
print('PASS: wrong runtime rejected')
