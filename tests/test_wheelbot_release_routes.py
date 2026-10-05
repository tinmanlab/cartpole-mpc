"""Check Playwright route callback arguments without launching a browser."""
from pathlib import Path
import ast
import json

source=Path(__file__).with_name('test_wheelbot_release_browser.py')
tree=ast.parse(source.read_text())
functions=[node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=='json_response_handler']
assert len(functions)==1,'Missing argument-safe JSON route handler'
namespace={}
exec(compile(ast.Module(body=functions,type_ignores=[]),str(source),'exec'),namespace)
class Route:
    def __init__(self):self.kwargs=None
    def fulfill(self,**kwargs):self.kwargs=kwargs
for request_args in [(),(object(),)]:
    route=Route();payload=json.dumps({'steps':249,'baselineSha256':'rejected-profile'})
    namespace['json_response_handler'](payload)(route,*request_args)
    assert route.kwargs['body']==payload
    assert route.kwargs['content_type']=='application/json'
    assert json.loads(route.kwargs['body'])['steps']==249
print('Route request argument cannot replace the captured JSON response PASS')
