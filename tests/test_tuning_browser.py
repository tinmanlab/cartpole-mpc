"""Read-only evidence UI: never pretend native SMAC runs inside the browser."""
from pathlib import Path
import functools,http.server,threading,json
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1]
class Quiet(http.server.SimpleHTTPRequestHandler):
 def log_message(self,*args):pass

def evidence_response(mode):
    # Exactly one positional argument: Playwright supplies a Request when a
    # callback has a second positional parameter, even if it has a default.
    body=json.dumps({'experimentValid':True,'mode':mode,'campaigns':[]})
    def respond(route):
        route.fulfill(content_type='application/json',body=body)
    return respond

server=http.server.ThreadingHTTPServer(('127.0.0.1',0),functools.partial(Quiet,directory=str(ROOT)))
threading.Thread(target=server.serve_forever,daemon=True).start()
try:
 with sync_playwright() as p:
    b=p.chromium.launch(headless=True);page=b.new_page(viewport={'width':1440,'height':1100});errors=[]
    page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto(f'http://127.0.0.1:{server.server_port}/index.html',wait_until='networkidle');page.wait_for_function('window.__labReady')
    assert page.locator('#loadTuningComparison').count()==1,'Missing offline-tuner evidence section'
    page.evaluate('window.controlLab.pause()');before=page.evaluate('window.controlLab.getState()')
    page.locator('#tuningDetails').evaluate('e=>e.open=true');page.click('#loadTuningComparison')
    page.wait_for_function('document.querySelector("#tuningComparisonStatus").dataset.state!=="loading"')
    assert page.locator('#tuningComparisonStatus').get_attribute('data-state')=='loaded'
    r=json.loads((ROOT/'evidence/sequential_tuning.json').read_text());assert r['experimentValid']
    assert page.locator('#tuningComparison tbody tr').count()==5
    text=page.locator('#tuningDetails').inner_text()
    assert '오프라인' in text and '108' in text and '216' in text and '기본값' in text
    for method in ['grid_budget','random_full','random_racing','smac_racing']:assert method in text
    after=page.evaluate('window.controlLab.getState()')
    for key in ['controller','observer','t','truth','estimate','terminalCost']:assert before[key]==after[key],key
    page.locator('#tuningDetails').screenshot(path=str(ROOT/'evidence/sequential_tuning_browser.png'))
    # Completed rejection/partial comparison must not become fake 0/12 successes.
    for mode in ['completed-no-admissible','completed-partial-admissibility']:
        response=evidence_response(mode)
        page.route('**/evidence/sequential_tuning.json',response)
        page.click('#loadTuningComparison');page.wait_for_function('document.querySelector("#tuningComparisonStatus").dataset.state==="error"')
        assert page.locator('#tuningComparison tbody tr').count()==0
        page.unroute('**/evidence/sequential_tuning.json',response)
    page.route('**/evidence/sequential_tuning.json',lambda route:route.fulfill(status=404,body='not found'))
    page.click('#loadTuningComparison');page.wait_for_function('document.querySelector("#tuningComparisonStatus").dataset.state==="error"')
    assert page.locator('#tuningComparison tbody tr').count()==0
    assert not errors,errors
    out={'schema':'cartpole-sequential-tuning-browser/v1','passed':True,'offlineReceiptLabel':True,'equalAndExtraBudgetLabels':True,'mainRuntimeUnchanged':True,'missingReceiptClearsStaleResult':True,'noResultNotRenderedAsZeroSuccess':True,'pageErrors':errors,'publicDeployment':False}
    (ROOT/'evidence/sequential_tuning_browser.json').write_text(json.dumps(out,indent=2)+'\n');b.close();print(json.dumps(out))
finally:server.shutdown()
