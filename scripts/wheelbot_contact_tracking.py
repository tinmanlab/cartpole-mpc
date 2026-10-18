"""Pure scheduled trajectory-error feedback. No plant, inverse, or optimizer imports."""
import json
import hashlib
from pathlib import Path
import numpy as np

INDICES=[0,1,2,3,4,6,7,8,9,10,11]

def selected_derivatives_validated(checks,refinements,condition,expected_columns=None):
    """Finite local consistency on the selected nominal branch, not stability."""
    if expected_columns is None:
        expected_columns={(sign,k,kind,j) for sign in [-1,1] for k in range(250)
                          for kind,n in [('x',11),('u',3)] for j in range(n)}
    def key(row):return tuple(row[k] for k in ['sign','step','kind','column'])
    def finite_error(value):return np.isfinite(value) and value>=0
    try:
        if not np.isfinite(condition) or not 0<condition<=1e10:return False
        mapped={key(r):r for r in refinements}
        columns={key(r) for r in checks}
        if len(mapped)!=len(refinements) or len(columns)!=len(checks) or columns!=set(expected_columns):return False
        needed=set()
        for row in checks:
            branches=row['branches'];error=row['relative_refinement_error']
            if len(branches)!=2 or not finite_error(error):return False
            nominal=[b['contacts'] is True and b['constraints'] is True for b in branches]
            if not nominal[1]:return False
            if nominal[0] and error<.1:continue
            column=key(row);needed.add(column)
            finer=mapped.get(column)
            if finer is None or finer['same_branch']!=[True,True]:return False
            if any(type(v) is not bool for v in finer['same_branch']):return False
            if not finite_error(finer['relative_error']) or finer['relative_error']>=.1:return False
        return set(mapped)==needed
    except (KeyError,TypeError,ValueError):
        return False

class TrackingController:
    def __init__(self,profile,active_identities):
        try:
            if profile['branchDiagnostics']['selected_derivatives_validated'] is not True:raise ValueError('Selected derivatives not admitted')
            if profile['schema']!='wheelbot-contact-tracking/v1':raise ValueError('Invalid schema')
            if profile['controlledIndices']!=INDICES:raise ValueError('Unknown state index mapping')
            if profile['dt']!=.002 or profile['steps']!=250:raise ValueError('Invalid schedule')
            for key in ['model_sha256','cost_sha256','trace_sha256','protocol_sha256']:
                if profile['identities'][key]!=active_identities[key]:raise ValueError('Identity mismatch: '+key)
            self.limits=np.array(profile['limitsNm'],dtype=float)
            if not np.array_equal(self.limits,[16,16,1.7]):raise ValueError('Invalid torque limits')
            self.rows={}
            for sign in [-1,1]:
                row=profile['trajectories'][str(sign)]
                values=[np.array(row[key],dtype=float) for key in ['ref','u','K']]
                for value,shape in zip(values,[(251,12),(250,3),(250,3,11)]):
                    if value.shape!=shape or not np.isfinite(value).all():raise ValueError('Malformed trajectory')
                if np.any(np.abs(values[1])>self.limits):raise ValueError('Reference exceeds torque limits')
                self.rows[sign]=values
        except (KeyError,TypeError,IndexError) as error:
            raise ValueError('Malformed profile or missing active identity') from error

    def command(self,state,index,sign,stage='tracking',feedback=True):
        if stage!='tracking' or type(index) is not int or not 0<=index<250 or type(sign) is not int or sign not in self.rows or type(feedback) is not bool:
            raise ValueError('Invalid stage, sign, or expired time index')
        x=np.asarray(state,dtype=float)
        if x.shape!=(12,) or not np.isfinite(x).all():raise ValueError('Invalid state')
        ref,u,K=self.rows[sign]
        torque=u[index]-K[index]@(x-ref[index])[INDICES] if feedback else u[index].copy()
        if not np.isfinite(torque).all():raise ValueError('Nonfinite torque')
        torque=np.clip(torque,-self.limits,self.limits)
        if np.any(np.abs(torque)>self.limits):raise ValueError('Torque limit violation')
        return torque

def load_controller(path,active_identities,*,active_xml):
    # Hash once at loading, never in the command loop. Other active identities
    # must likewise come from the caller's current cost/trace/protocol files.
    if hashlib.sha256(Path(active_xml).read_bytes()).hexdigest()!=active_identities.get('model_sha256'):
        raise ValueError('Active XML does not match supplied plant identity')
    return TrackingController(json.loads(Path(path).read_text()),active_identities)
