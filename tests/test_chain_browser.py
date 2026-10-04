"""Actual HTTP browser N-chain check; no WebMCP shim or screenshot-only success."""
import functools,http.server,threading,json
from pathlib import Path
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1]
class Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self,*args):pass
server=http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(Quiet,directory=str(ROOT)))
threading.Thread(target=server.serve_forever,daemon=True).start()
try:
 with sync_playwright() as p:
    browser=p.chromium.launch(headless=True);page=browser.new_page(viewport={'width':1440,'height':1000});errors=[]
    page.on('pageerror',lambda e:errors.append(str(e)))
    response=page.goto(f'http://127.0.0.1:{server.server_port}/chain.html',wait_until='networkidle')
    assert response.status==200,'N-chain page missing'
    page.wait_for_function('window.chainLab?.ready')
    snap=page.evaluate('window.chainLab.run(600)')
    assert snap['poles']==2 and snap['steps']==600 and snap['status']=='completed',snap
    assert len(snap['truth'])==6 and snap['physics']['backend']=='mujoco-wasm'
    assert len(snap['geometry']['links'])==2 and snap['observer']=='steady_kf'
    for n in ['3','4']:
        page.select_option('#poles',n);page.select_option('#controller','lqr');page.wait_for_function('window.chainLab.ready')
        snap=page.evaluate('window.chainLab.run(600)');assert snap['steps']==600 and snap['status']=='completed',snap
    page.select_option('#poles','8');page.wait_for_function('window.chainLab.ready')
    snap=page.evaluate('window.chainLab.getState()');assert snap['status']=='design-rejected' and snap['steps']==0
    assert len(snap['geometry']['links'])==8
    assert '설계 거부' in page.locator('#outcomes tr').last.inner_text(), 'Rejected design must not masquerade as an executed rollout'
    page.select_option('#poles','2');page.select_option('#controller','mpc');page.wait_for_function('window.chainLab.ready')
    page.evaluate('window.chainLab.run(120)')
    page.screenshot(path=str(ROOT/'evidence/chain_browser.png'),full_page=True)
    assert page.evaluate('document.documentElement.scrollWidth-innerWidth')<=1
    assert not errors,errors
    out={'schema':'cartpole-chain-browser/v1','actualWasm':True,'N2MpcEncoderLoop':True,'N3N4LqrEncoderLoop':True,'N8FailClosed':True,'pageErrors':errors,'passed':True,'publicDeployed':False}
    (ROOT/'evidence/chain_browser.json').write_text(json.dumps(out,indent=2)+'\n');browser.close();print(json.dumps(out))
finally:server.shutdown()
