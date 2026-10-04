"""Shared terminal accounting for the two existing tuning CLIs.
No selection, optimizer, simulator or performance-policy changes.
"""
from __future__ import annotations
import hashlib,json
from pathlib import Path


def summarize_test(campaign, panel, *, skipped=False):
    rows=campaign.get('testRows',[])
    rejected=campaign['selection']['configuration'] is None
    if rejected or skipped:
        if rows:
            raise RuntimeError('Unevaluated recommendation unexpectedly contains test rows')
        status='not-evaluated-no-admissible' if rejected else 'not-evaluated-smoke'
        return {'testStatus':status,'actualTestEvaluations':0,
                'primaryTaskSuccesses':None,'boundaryTaskSuccesses':None,
                'primaryMeanTaskScore':None}
    expected={r['id'] for r in panel}
    if len(rows)!=len(expected) or {r['caseId'] for r in rows}!=expected:
        raise RuntimeError('Incomplete or duplicated locked test panel; reconcile before finalizing')
    groups={r['id']:r['group'] for r in panel}
    if any(r['group']!=groups[r['caseId']] for r in rows):
        raise RuntimeError('Test row group differs from the declared panel')
    primary=[r for r in rows if r['group']=='primary']
    complete=primary and all(r['outcome']=='completed' for r in primary)
    return {'testStatus':'evaluated','actualTestEvaluations':len(rows),
            'primaryTaskSuccesses':sum(bool(r['taskPassed']) for r in primary),
            'boundaryTaskSuccesses':sum(bool(r['taskPassed']) for r in rows if r['group']=='boundary'),
            'primaryMeanTaskScore':sum(r['fullScore'] for r in primary)/len(primary) if complete else None}


def _valid_no_result(report):
    campaigns=report.get('campaigns',[])
    return bool(campaigns) and report.get('experimentValid') is True and report.get('sourcesStable') is True and report.get('actualTestEvaluations')==0 and all(
        c.get('selection',{}).get('configuration','missing') is None
        and c['selection'].get('eligible') is False
        and c.get('testStatus')=='not-evaluated-no-admissible'
        and c.get('actualTestEvaluations')==0
        and all(c.get(k,'missing') is None for k in ['primaryTaskSuccesses','boundaryTaskSuccesses','primaryMeanTaskScore'])
        for c in campaigns)


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def complete_no_result(out, report_path, report):
    """Keep incomplete comparisons out of legacy success tables; mark safe retries."""
    campaigns=report.get('campaigns',[])
    if campaigns and any(c['selection']['configuration'] is None for c in campaigns):
        # Legacy tables accept only mode=frozen-full and would otherwise coerce
        # null success counts to zero. The typed JSON remains the result owner.
        report['executionMode']=report['mode']
        report['mode']='completed-no-admissible' if all(c['selection']['configuration'] is None for c in campaigns) else 'completed-partial-admissibility'
        report['comparisonAvailable']=False
        report_path.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    if not _valid_no_result(report):
        return
    lock_path=out/'recommendations-locked.json'
    lock=json.loads(lock_path.read_text())
    if lock!=report['freshTestLock'] or lock.get('testEvaluations')!=0:
        raise RuntimeError('Completion lock/report mismatch')
    expected=[{'campaign':c['campaign'],'configuration':None} for c in report['campaigns']]
    if lock.get('recommendations')!=expected:
        raise RuntimeError('Completion recommendations differ from lock')
    marker={'schema':'cartpole-no-admissible-completion/v1',
            'sourceSha256':report['sourceSha256'],'reportSha256':_sha(report_path),
            'lockSha256':_sha(lock_path),'actualTestEvaluations':0}
    temporary=out/'finalization-complete.json.tmp'
    temporary.write_text(json.dumps(marker,indent=2,allow_nan=False)+'\n')
    temporary.replace(out/'finalization-complete.json')


def reuse_completed_no_result(out, report_path, hashes):
    """Never replay a complete rejection, and never bless an incomplete test run."""
    marker_path=out/'finalization-complete.json'
    if not marker_path.exists():
        return False
    try:
        marker=json.loads(marker_path.read_text())
        lock_path=out/'recommendations-locked.json'
        report=json.loads(report_path.read_text())
        lock=json.loads(lock_path.read_text())
        valid=(marker.get('schema')=='cartpole-no-admissible-completion/v1'
               and marker.get('sourceSha256')==hashes==report.get('sourceSha256')
               and marker.get('reportSha256')==_sha(report_path)
               and marker.get('lockSha256')==_sha(lock_path)
               and marker.get('actualTestEvaluations')==0
               and lock==report.get('freshTestLock')
               and lock.get('recommendations')==[{'campaign':c['campaign'],'configuration':None} for c in report.get('campaigns',[])]
               and _valid_no_result(report))
    except (OSError,ValueError,KeyError,TypeError) as exc:
        raise RuntimeError('Invalid completed no-result receipt; reconcile before retry') from exc
    if not valid:
        raise RuntimeError('Completed no-result source/receipt changed; reconcile before retry')
    print('REUSED completed no-admissible result; zero new searches/tests',flush=True)
    return True
