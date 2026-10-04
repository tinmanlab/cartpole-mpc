"""Bounded CLI for the same frozen study: one seed's campaigns, then final holdout.
Completed campaigns are reused by exact source identity. No queue/service/background agent.
"""
from __future__ import annotations
import argparse,fcntl,gzip,inspect,json,time,importlib.metadata as md
from pathlib import Path
from sequential_tuning import ROOT,PROTOCOL_HASH,load_protocol,sha,PlantBridge,EvaluationLog,do_campaign,test_cases,paired_statistics,AlgorithmConfigurationFacade,Intensifier

FILES=['scripts/sequential_tuning.py','scripts/run_tuning_campaign.py','scripts/tuning_bridge.mjs','src/design_study.js','src/engine.js','src/plant.js','src/qp.js','src/mujoco_backend.mjs','assets/cartpole.xml','tests/fixtures/sequential_tuning.json','tests/fixtures/design_study.json']

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--seed',type=int);ap.add_argument('--grid-reference',action='store_true');ap.add_argument('--finalize',action='store_true');args=ap.parse_args()
    if sum([args.seed is not None,args.grid_reference,args.finalize])!=1:ap.error('Choose one seed, grid-reference, or finalize')
    p,m=load_protocol();out=ROOT/'test-results/sequential-tuning';out.mkdir(parents=True,exist_ok=True)
    hashes={f:sha(ROOT/f) for f in FILES};header=out/'source-identity.json'
    if header.exists():
        if json.loads(header.read_text())!=hashes:raise RuntimeError('Source identity changed; cannot resume this frozen campaign')
    else:header.write_text(json.dumps(hashes,indent=2))
    lockfile=open(out/'execution.lock','w');fcntl.flock(lockfile,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if md.version('smac')!=p['smac']['version'] or md.version('ConfigSpace')!=p['smac']['configspaceVersion']:raise RuntimeError('Dependency versions changed')
    with PlantBridge(ROOT) as bridge:
        (out/'physics-info.json').write_text(json.dumps(bridge.info,indent=2))
        if not args.finalize:
            if (out/'recommendations-locked.json').exists():raise RuntimeError('No searches after holdout lock')
            if args.grid_reference:jobs=[('grid_exhaustive',p['optimizerSeeds'][0],p['grid']['exhaustiveRollouts'])]
            else:
                if args.seed not in p['optimizerSeeds']:raise ValueError('Unknown frozen optimizer seed')
                i=p['optimizerSeeds'].index(args.seed);order=p['methods'][i%4:]+p['methods'][:i%4]
                jobs=[(method,args.seed,p['searchRolloutBudget']) for method in order]
            for method,seed,budget in jobs:
                name=f'{method}-{seed}';receipt=out/(name+'.json');raw=out/('raw-'+name+'.jsonl.gz')
                if receipt.exists():
                    if not raw.exists():raise RuntimeError('Completed campaign missing raw evidence')
                    print('REUSED completed same-source campaign '+name,flush=True);continue
                if raw.exists():raise RuntimeError('Incomplete raw campaign requires explicit reconciliation: '+name)
                log=EvaluationLog(bridge,p,raw)
                try:do_campaign(p,m,method,seed,log,ROOT,out,budget)
                finally:log.close()
                if any(sha(ROOT/f)!=h for f,h in hashes.items()):raise RuntimeError('Source changed during campaign')
                (ROOT/'test-results/tuner-progress.json').write_text(json.dumps({'lastCompleted':name,'sourceStable':True,'testEvaluations':0}))
            return
        names=[f'{method}-{seed}' for seed in p['optimizerSeeds'] for method in p['methods']]+[f'grid_exhaustive-{p["optimizerSeeds"][0]}']
        if not all((out/(n+'.json')).exists() for n in names):raise RuntimeError('All frozen searches must finish before any final test')
        if (out/'recommendations-locked.json').exists():raise RuntimeError('Final test previously started; reconcile before repeating')
        campaigns=[json.loads((out/(name+'.json')).read_text()) for name in names]
        lock={'recommendations':[{'campaign':c['campaign'],'configuration':c['selection']['configuration']} for c in campaigns],'testEvaluations':0,'selectionComplete':True}
        (out/'recommendations-locked.json').write_text(json.dumps(lock,indent=2));bridge.lock([x['configuration'] for x in lock['recommendations'] if x['configuration'] is not None])
        log=EvaluationLog(bridge,p,out/'raw-final-tests.jsonl.gz')
        try:
            for c in campaigns:
                t0=time.perf_counter();cfg=c['selection']['configuration'];c['testRows']=[]
                if cfg is not None:
                    for test in test_cases(p,m):
                        row=log.evaluate(cfg,test['id'],'test',c['campaign']);row['group']=test['group'];c['testRows'].append(row)
                c['testWallSeconds']=time.perf_counter()-t0;c['primaryTaskSuccesses']=sum(r['taskPassed'] for r in c['testRows'] if r['group']=='primary');c['boundaryTaskSuccesses']=sum(r['taskPassed'] for r in c['testRows'] if r['group']=='boundary')
                c['primaryMeanTaskScore']=sum(r['fullScore'] for r in c['testRows'] if r['group']=='primary')/12 if len(c['testRows'])==18 and all(r['outcome']=='completed' for r in c['testRows'] if r['group']=='primary') else None
                (out/(c['campaign']+'.json')).write_text(json.dumps(c,indent=2,allow_nan=False)+'\n')
                print(json.dumps({'tested':c['campaign'],'primaryPassed':c['primaryTaskSuccesses'],'primaryScore':c['primaryMeanTaskScore']}),flush=True)
        finally:log.close()
        # Merge complete raw streams in evaluation-phase order for a separate independent verifier.
        total=0
        with gzip.open(out/'raw-evaluations.jsonl.gz','wt',encoding='utf-8') as target:
            for path in [out/('raw-'+name+'.jsonl.gz') for name in names]+[out/'raw-final-tests.jsonl.gz']:
                with gzip.open(path,'rt',encoding='utf-8') as source:
                    for line in source:
                        row=json.loads(line);row['recordIndex']=total;total+=1;target.write(json.dumps(row,separators=(',',':'),allow_nan=False)+'\n')
        stable=all(sha(ROOT/f)==h for f,h in hashes.items())
        versions={k:md.version(k) for k in ['smac','ConfigSpace','scikit-learn','numpy','scipy']}
        report={'schema':'cartpole-sequential-tuning/v1','mode':'frozen-full','sourceSha256':hashes,'sourcesStable':stable,'protocolSha256':PROTOCOL_HASH,'versions':versions,
          'upstreamSourceSha256':{'facade':sha(Path(inspect.getfile(AlgorithmConfigurationFacade))),'intensifier':sha(Path(inspect.getfile(Intensifier)))},'physics':bridge.info['physics'],'fit':bridge.info['fit'],
          'campaigns':[{k:v for k,v in c.items() if k not in ['trainingRows','validationRows','testRows']} for c in campaigns],'pairedStatistics':paired_statistics(campaigns,p),'freshTestLock':lock,
          'sumMeasuredCampaignSeconds':sum(c['searchWallSeconds']+c['validationWallSeconds']+c['testWallSeconds'] for c in campaigns),'actualEvaluations':total,'experimentValid':stable,'defaultPromoted':False,'hardware':'NOT_EVALUATED',
          'execution':'Same frozen algorithms executed in bounded seed-sized CLI batches because remote jobs cap at 600 seconds. No evaluator cache across methods. Exact-source completed campaigns can be resumed. All searches/validation complete before final holdout.',
          'developmentAttempts':'Separate pre-holdout smoke exposed non-JSON internal RF bounds. A monolithic run was cancelled before any test after confirming the remote job cap; its partial logs are retained outside this final benchmark and not treated as independent final repeats. No search domain, score, seed or threshold was changed.',
          'scope':['Actual SMAC3 random-forest EI + its standard Intensifier; random_racing sets proposal randomization to1 in the same infrastructure.','Conditional horizon exists only for MPC. Same effort/process domain, fixed rollout length, same physical task/command gates.','Same 108 search calls including12 shared baseline calls; validation/test calls and costs separate. No incomplete training candidate can be recommended.','Additional grid216 is finite reference, not a continuous optimum. Randomized grid ordering is repeated for matched search budgets.','Ordinal loss preserves full-three-instance failure/task/score ranking under verified bounds; partial racing estimates are not full-set certificates.','Five optimizer seeds on a fixed common test panel: conditional pilot inference, not universal speedup, safety or a rare-failure guarantee.','Known training/validation templates are development information; new test seeds do not establish a new physical domain.']}
        (ROOT/'evidence/sequential_tuning.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print(json.dumps({'evaluations':total,'statistics':report['pairedStatistics'],'valid':stable},indent=2))
        if not stable:raise RuntimeError('Source mismatch')

if __name__=='__main__':main()
