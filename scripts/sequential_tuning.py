"""Optional actual SMAC3/ConfigSpace tuning of the existing WASM task evaluator.

Four equal-search-budget methods, conditional configuration space and a separate
finite exhaustive-grid reference. No custom optimizer, racing rule or simulator.
"""
from __future__ import annotations
import argparse,copy,gzip,hashlib,importlib.metadata as md,inspect,itertools,json,math,os,selectors,subprocess,time,warnings
from pathlib import Path
from collections import defaultdict,Counter
import numpy as np
from ConfigSpace import ConfigurationSpace,Configuration,Categorical,Float,EqualsCondition
from smac import AlgorithmConfigurationFacade,Scenario
from smac.initial_design.sobol_design import SobolInitialDesign
from smac.intensifier.intensifier import Intensifier
from smac.random_design.probability_design import ProbabilityRandomDesign
from smac.acquisition.maximizer.local_and_random_search import LocalAndSortedRandomSearch
from smac.runhistory.dataclasses import TrialInfo,TrialValue
ROOT=Path(__file__).resolve().parents[1]
PROTOCOL_HASH='9ca92d14fceea5b847d75175e8edc4f51837eb920d1d35b5361929a5387accf0'

def sha(path:Path)->str:return hashlib.sha256(path.read_bytes()).hexdigest()
def plain(c):
    d=dict(c);out={'controller':str(d['controller']),'observer':str(d['observer']),'effort':float(d['effort']),'process':float(d['process'])}
    if 'horizon' in d:out['horizon']=int(d['horizon'])
    return out

def configuration_key(c):return json.dumps(plain(c),sort_keys=True,separators=(',',':'))

def load_protocol():
    path=ROOT/'tests/fixtures/sequential_tuning.json'
    if sha(path)!=PROTOCOL_HASH:raise ValueError('Frozen sequential-tuning protocol changed')
    p=json.loads(path.read_text());mp=ROOT/p['taskManifest']
    if sha(mp)!=p['taskManifestSha256']:raise ValueError('Base task changed')
    m=json.loads(mp.read_text())
    seeds=[m['calibration']['seed']]+[t['seed'] for phase in ['training','validation'] for t in m[phase]]+[s for row in p['freshTestSeedRows'] for s in row]
    if len(set(seeds))!=len(seeds):raise ValueError('Experiment seed sets overlap')
    # Analytic task-score bound in the evaluated state envelope, not a fitted clipping constant.
    o=m['task']['objective'];t=m['task'];max_score=o['position']['weight']*(2*t['railLimit_m']/o['position']['scale'])**2+o['angle']['weight']*(t['angleEnvelope_rad']/o['angle']['scale'])**2+o['force']['weight']*(t['forceLimit_N']/o['force']['scale'])**2+o['deltaForce']['weight']*(2*t['forceLimit_N']/o['deltaForce']['scale'])**2
    if max_score>p['loss']['taskScoreUpperBound']:raise ValueError('Ordinal score bound not established')
    return p,m

def configuration_space(p,seed):
    d=p['domain'];cs=ConfigurationSpace(seed=seed)
    ctl=Categorical('controller',d['controllers'],default='lqr');obs=Categorical('observer',d['observers'],default='kf')
    effort=Float('effort',tuple(d['effortBounds']),default=1.,log=True);process=Float('process',tuple(d['processBounds']),default=1.,log=True)
    horizon=Categorical('horizon',d['mpcHorizons'],default=d['baselineHorizon'])
    cs.add([ctl,obs,effort,process,horizon]);cs.add(EqualsCondition(horizon,ctl,'hard_mpc'));return cs

def initial_configurations(p):
    return [{'controller':c,'observer':o,'effort':1.,'process':1.,**({'horizon':p['domain']['baselineHorizon']} if c=='hard_mpc' else {})} for c in p['domain']['controllers'] for o in p['domain']['observers']]

def grid_configurations(p):
    return [{'controller':c,'observer':o,'effort':float(e),'process':float(q),**({'horizon':h} if c=='hard_mpc' else {})} for c in p['domain']['controllers'] for o in p['domain']['observers'] for e in p['grid']['effort'] for q in p['grid']['process'] for h in (p['grid']['mpcHorizons'] if c=='hard_mpc' else [None])]

def encode_loss(row,p):
    if row['outcome'] not in ['completed','solver-rejected','envelope-failure']:raise ValueError('Unexpected evaluator error; do not turn an implementation bug into a penalty')
    hard=row['outcome']!='completed';task=not row['taskPassed'];score=0. if hard else row['fullScore']
    if hard and (row['fullScore'] is not None or row['taskPassed']):raise ValueError('Failed run has fabricated full score/success')
    if not isinstance(score,(int,float)) or not math.isfinite(score) or not 0<=score<=p['loss']['taskScoreUpperBound']:raise ValueError('Task score exceeds proven bound; no clipping')
    return p['loss']['hardFailureWeight']*hard+p['loss']['taskFailureWeight']*task+score/p['loss']['taskScoreUpperBound']

class PlantBridge:
    def __init__(self,root=ROOT):
        self.stderr_file=open(root/'test-results/tuner-bridge-stderr.log','a')
        self.process=subprocess.Popen(['node','scripts/tuning_bridge.mjs'],cwd=root,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.stderr_file,text=True,bufsize=1)
        self.info=self.call({'op':'info'})
    def call(self,data):
        self.process.stdin.write(json.dumps(data,allow_nan=False)+'\n');self.process.stdin.flush()
        with selectors.DefaultSelector() as s:
            s.register(self.process.stdout,selectors.EVENT_READ)
            if not s.select(timeout=90):raise TimeoutError('WASM evaluation did not return; no duplicate replay')
        text=self.process.stdout.readline()
        if not text:raise RuntimeError('WASM process closed; inspect its stderr receipt')
        r=json.loads(text)
        if not r.get('ok'):raise RuntimeError(r.get('error','Unknown bridge error'))
        return r.get('result',r)
    def evaluate(self,configuration,case_id,phase):return self.call({'op':'evaluate','configuration':plain(configuration),'caseId':case_id,'phase':phase})
    def lock(self,configs):return self.call({'op':'lock','configurations':[plain(c) for c in configs]})
    def __enter__(self):return self
    def __exit__(self,*exc):
        try:
            if self.process.poll() is None:self.call({'op':'close'})
        finally:
            if self.process.poll() is None:
                try:self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(timeout=5)
            self.process.stdin.close();self.process.stdout.close();self.stderr_file.close()

def make_smac(p,m,seed,directory,method):
    cs=configuration_space(p,seed)
    scenario=Scenario(cs,name=f'{method}-{seed}',deterministic=True,n_trials=p['searchRolloutBudget'],instances=[t['id'] for t in m['training']],instance_features={t['id']:[float(i==j) for j in range(len(m['training']))] for i,t in enumerate(m['training'])},seed=seed,n_workers=1,output_directory=directory,crash_cost=100.)
    init=SobolInitialDesign(scenario,n_configs=p['smac']['initialSobolConfigurations'],max_ratio=1.,seed=seed)
    intensifier=Intensifier(scenario,max_config_calls=len(m['training']),seed=seed)
    random=ProbabilityRandomDesign(probability=1. if method=='random_racing' else p['smac']['randomProbability'],seed=seed)
    maximizer=LocalAndSortedRandomSearch(cs,challengers=p['smac']['acquisitionChallengers'],seed=seed)
    selector=AlgorithmConfigurationFacade.get_config_selector(scenario,retrain_after=p['smac']['retrainAfter'])
    return AlgorithmConfigurationFacade(scenario,target_function=None,initial_design=init,intensifier=intensifier,random_design=random,acquisition_maximizer=maximizer,config_selector=selector,logging_level=False,overwrite=True)

def certified_groups(rows,m,phase):
    if phase not in ['training','validation']:raise ValueError('Test cannot select configurations')
    groups=defaultdict(list)
    for row in rows:
        if row['phase']!=phase:raise ValueError('Wrong phase in selector')
        groups[configuration_key(row['configuration'])].append(row)
    expected={t['id'] for t in m[phase]};out=[]
    for key,rr in groups.items():
        if len(rr)!=len(expected) or {r['caseId'] for r in rr}!=expected:continue
        hard=sum(r['outcome']!='completed' for r in rr);task=sum(not r['taskPassed'] for r in rr);score=float(np.mean([r['fullScore'] for r in rr])) if not hard else None
        out.append({'configuration':plain(rr[0]['configuration']),'hardFailures':hard,'taskFailures':task,'score':score,'key':key})
    return out

def is_base(c):return c['effort']==c['process']==1 and c.get('horizon',30)==30

def ranking(a):return (a['hardFailures'],a['taskFailures'],a['score'] if a['score'] is not None else float('inf'),0 if is_base(a['configuration']) else 1,a['key'])

def choose_final(validation_rows,m):
    groups=certified_groups(validation_rows,m,'validation');eligible=[g for g in groups if not g['hardFailures'] and not g['taskFailures']]
    if not eligible:return {'configuration':None,'eligible':False,'reason':'No fully successful validation candidate','groups':groups}
    best=min(g['score'] for g in eligible);near=[g for g in eligible if g['score']<=best*(1+m['selection']['crossPairTieRelative'])]
    def policy(g):
        c=g['configuration'];order=next(p['simplicityOrder'] for p in m['pairs'] if p['controller']==c['controller'] and p['observer']==c['observer'])
        return order,g['score'],0 if is_base(c) else 1,g['key']
    winner=min(near,key=policy)
    return {**winner,'eligible':True,'groups':groups,'testUsedForSelection':False,'scope':'Validation policy within searched configurations; not a global optimum'}

class EvaluationLog:
    def __init__(self,bridge,p,path):self.bridge=bridge;self.p=p;self.fp=gzip.open(path,'wt',encoding='utf-8');self.count=0
    def close(self):self.fp.close()
    def evaluate(self,c,case_id,phase,campaign):
        start=time.perf_counter();row=self.bridge.evaluate(c,case_id,phase);wall=time.perf_counter()-start
        # Verify score arithmetic directly from raw task series before passing anything to a tuner.
        if row['appliedSteps']:
            a=np.asarray(row['auditSeries']);terms=a[:,:4]**2*np.array([16.,20.,.002,.005]);score=float(np.sum(np.mean(terms,axis=0)))
            if abs(score-row['partialScore'])>1e-9:raise AssertionError('Task score reconstruction mismatch')
        row['loss']=encode_loss(row,self.p);row['evaluationWallSeconds']=wall;row['campaign']=campaign;row['recordIndex']=self.count;self.count+=1
        self.fp.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n');self.fp.flush()
        return {k:v for k,v in row.items() if k not in ['auditSeries','trace','design']}

def do_campaign(p,m,method,seed,log,root,out_dir,budget=None):
    budget=budget or p['searchRolloutBudget'];campaign=f'{method}-{seed}';t0=time.perf_counter();rows=[];checkpoints=[];asked=[];seen=set();tuner_s=0.;duplicate_asks=0
    optimizer=make_smac(p,m,seed,out_dir/campaign,method) if method in ['smac_racing','random_racing'] else None
    cs=configuration_space(p,seed);rng=np.random.default_rng(seed);tuner_s=time.perf_counter()-t0
    def evaluate(c,case_id,origin,info=None):
        nonlocal tuner_s
        c=plain(c);key=(configuration_key(c),case_id)
        if key in seen:raise AssertionError('Repeated deterministic training trial charged twice')
        seen.add(key);row=log.evaluate(c,case_id,'training',campaign);row['origin']=origin;rows.append(row)
        if optimizer:
            start=time.perf_counter()
            if info is None:info=TrialInfo(config=Configuration(optimizer.scenario.configspace,values=c),instance=case_id,seed=0)
            optimizer.tell(info,TrialValue(cost=row['loss'],time=row['evaluationWallSeconds']),save=False)
            tuner_s+=time.perf_counter()-start
        if len(rows) in p['checkpoints']:
            certified=sorted(certified_groups(rows,m,'training'),key=ranking);good=[q for q in certified if not q['hardFailures'] and not q['taskFailures']]
            checkpoints.append({'rollouts':len(rows),'wallSeconds':time.perf_counter()-t0,'tunerSeconds':tuner_s,'fullyEvaluatedConfigurations':len(certified),'bestCertifiedScore':min((q['score'] for q in good),default=None),'bestCertifiedConfiguration':min(good,key=lambda q:q['score'])['configuration'] if good else None})
    # Equal prior information and equal charges. Fresh actual evaluations, no cross-campaign cache.
    for c in initial_configurations(p):
        for t in m['training']:evaluate(c,t['id'],'shared-baseline')
    if optimizer:
        while len(rows)<budget:
            start=time.perf_counter();info=optimizer.ask();tuner_s+=time.perf_counter()-start
            c=plain(info.config);key=(configuration_key(c),info.instance)
            if key in seen:
                # The deterministic instance is fixed by the task seed, regardless of an optimizer seed field.
                # Duplicate asks indicate a wrong integration, not a reason to replay hidden extra work.
                duplicate_asks+=1;raise AssertionError(f'Duplicate SMAC instance: {key}; seed={info.seed}')
            if info.instance not in {t['id'] for t in m['training']}:raise AssertionError('SMAC proposed nontraining case')
            origin=str(info.config.origin);asked.append({'configuration':c,'instance':info.instance,'smacSeed':info.seed,'origin':origin})
            evaluate(c,info.instance,origin,info)
        optimizer.runhistory.save(out_dir/campaign/'runhistory.json')
        engine={'facade':type(optimizer).__name__,'model':type(optimizer._model).__name__,'intensifier':type(optimizer.intensifier).__name__,'modelModule':type(optimizer._model).__module__,'modelSourceSha256':sha(Path(inspect.getfile(type(optimizer._model)))),'originCounts':dict(Counter(x['origin'] for x in asked)),'runhistoryFinished':optimizer.runhistory.finished,'smacIncumbents':[plain(c) for c in optimizer.intensifier.get_incumbents()]}
    else:
        if method.startswith('grid'):
            baseline={configuration_key(c) for c in initial_configurations(p)};points=[c for c in grid_configurations(p) if configuration_key(c) not in baseline];rng.shuffle(points);iterator=iter(points)
        else:iterator=iter(lambda:plain(cs.sample_configuration()),None)
        while len(rows)<budget:
            start=time.perf_counter();c=next(iterator);tuner_s+=time.perf_counter()-start
            if any(configuration_key(c)==key[0] for key in seen):continue
            for t in m['training']:
                if len(rows)>=budget:break
                evaluate(c,t['id'],'grid' if method.startswith('grid') else 'random')
        engine={'sampler':'fixed logarithmic grid in randomized order' if method.startswith('grid') else 'ConfigSpace log-uniform random','fullEvaluationPerCandidate':True}
    search_wall=time.perf_counter()-t0
    certified=sorted(certified_groups(rows,m,'training'),key=ranking)
    shortlist=[g['configuration'] for g in certified[:p['finalSelection']['fullyEvaluatedTrainingShortlist']]]
    base=initial_configurations(p)[0]
    if configuration_key(base) not in {configuration_key(c) for c in shortlist}:shortlist.append(base)
    val=[];vstart=time.perf_counter()
    for c in shortlist:
        for t in m['validation']:val.append(log.evaluate(c,t['id'],'validation',campaign))
    choice=choose_final(val,m)
    allocation=Counter(configuration_key(r['configuration']) for r in rows)
    report={'method':method,'optimizerSeed':seed,'campaign':campaign,'searchBudget':budget,'actualSearchRollouts':len(rows),'actualSearchSteps':sum(r['appliedSteps'] for r in rows),
        'searchEvaluationSeconds':sum(r['evaluationWallSeconds'] for r in rows),'tunerSeconds':tuner_s,'searchWallSeconds':search_wall,'validationRollouts':len(val),'validationWallSeconds':time.perf_counter()-vstart,
        'uniqueConfigurations':len(allocation),'fullyEvaluatedConfigurations':len(certified),'allocationCounts':dict(Counter(allocation.values())),'duplicateAsks':duplicate_asks,
        'engine':engine,'checkpoints':checkpoints,'shortlist':shortlist,'selection':choice,'lockedBeforeTest':True,
        'trainingRows':rows,'validationRows':val,'testRows':[]}
    print(json.dumps({'campaign':campaign,'searchCalls':len(rows),'unique':len(allocation),'allocation':report['allocationCounts'],'tunerSeconds':tuner_s,'selected':choice.get('configuration'),'validationScore':choice.get('score')}),flush=True)
    (out_dir/(campaign+'.json')).write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report

def test_cases(p,m):return [{**t,'id':f'fresh-{i}-{s}','seed':s} for i,t in enumerate(m['test']) for s in p['freshTestSeedRows'][i]]

def paired_statistics(campaigns,p):
    from scipy.stats import bootstrap
    ref={c['optimizerSeed']:c for c in campaigns if c['method']=='grid_budget'};results=[]
    for method in ['random_full','random_racing','smac_racing']:
        rows=[];wall=[]
        for c in campaigns:
            if c['method']!=method:continue
            grid=ref[c['optimizerSeed']]
            a=[r for r in c['testRows'] if r['group']=='primary'];b=[r for r in grid['testRows'] if r['group']=='primary']
            complete=len(a)==len(b)==12 and all(r['outcome']=='completed' for r in a+b)
            if complete:
                baseline=float(np.mean([r['fullScore'] for r in b]));candidate=float(np.mean([r['fullScore'] for r in a]));rows.append({'seed':c['optimizerSeed'],'relativeTaskDifference':candidate/baseline-1,'pairedTaskSuccessDifference':sum(r['taskPassed'] for r in a)-sum(r['taskPassed'] for r in b)})
            wall.append(c['searchWallSeconds']/grid['searchWallSeconds'])
        values=np.array([q['relativeTaskDifference'] for q in rows]);interval=None
        if len(values)==len(p['optimizerSeeds']):
            if np.ptp(values)==0:interval=[float(values[0]),float(values[0])]
            else:
                boot=bootstrap((values,),np.mean,n_resamples=p['statistics']['pairedBootstrapResamples'],confidence_level=p['statistics']['confidenceLevel'],method='percentile',rng=np.random.default_rng(p['statistics']['bootstrapSeed']))
                interval=[float(boot.confidence_interval.low),float(boot.confidence_interval.high)]
        results.append({'method':method,'comparison':'paired grid_budget with same optimizer seed/common fixed test panel','pairs':rows,'meanRelativeTaskDifference':float(np.mean(values)) if len(values) else None,'conditionalPairedBootstrap95':interval,'meanSearchWallRatio':float(np.mean(wall)),'scope':p['statistics']['unit']})
    return results

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--smoke',action='store_true');args=parser.parse_args()
    p,m=load_protocol()
    if any(md.version(k)!=v for k,v in [('smac',p['smac']['version']),('ConfigSpace',p['smac']['configspaceVersion'])]):raise RuntimeError('Unverified optional tuner versions')
    out_dir=ROOT/('test-results/sequential-smoke' if args.smoke else 'test-results/sequential-tuning');out_dir.mkdir(parents=True,exist_ok=True)
    if (out_dir/'campaign-started.json').exists():raise RuntimeError('An existing campaign must be reconciled; do not silently overwrite evidence')
    files=['scripts/sequential_tuning.py','scripts/tuning_bridge.mjs','src/design_study.js','src/engine.js','src/plant.js','src/qp.js','src/mujoco_backend.mjs','assets/cartpole.xml','tests/fixtures/sequential_tuning.json','tests/fixtures/design_study.json']
    hashes={f:sha(ROOT/f) for f in files};start=time.perf_counter();started={'sourceSha256':hashes,'pid':os.getpid(),'mode':'smoke' if args.smoke else 'frozen-full'}
    (out_dir/'campaign-started.json').write_text(json.dumps(started,indent=2))
    campaigns=[]
    with PlantBridge(ROOT) as bridge:
        log=EvaluationLog(bridge,p,out_dir/'raw-evaluations.jsonl.gz')
        try:
            seeds=p['optimizerSeeds'][:1] if args.smoke else p['optimizerSeeds']
            for i,seed in enumerate(seeds):
                # Rotate execution order to avoid always assigning the same host phase to SMAC.
                order=p['methods'][i%4:]+p['methods'][:i%4]
                for method in order:
                    c=do_campaign(p,m,method,seed,log,ROOT,out_dir,budget=24 if args.smoke else None);campaigns.append(c)
                    (ROOT/'test-results/tuner-progress.json').write_text(json.dumps({'completedCampaigns':len(campaigns),'last':c['campaign'],'total':len(seeds)*4,'testEvaluations':0}))
            if not args.smoke:campaigns.append(do_campaign(p,m,'grid_exhaustive',p['optimizerSeeds'][0],log,ROOT,out_dir,budget=p['grid']['exhaustiveRollouts']))
            # Lock all recommendations BEFORE opening the new test panel.
            locks=[c['selection']['configuration'] for c in campaigns if c['selection']['configuration'] is not None]
            lock_receipt={'recommendations':[{ 'campaign':c['campaign'],'configuration':c['selection']['configuration']} for c in campaigns],'testEvaluations':0,'selectionComplete':True}
            (out_dir/'recommendations-locked.json').write_text(json.dumps(lock_receipt,indent=2))
            bridge.lock(locks)
            for c in campaigns:
                cfg=c['selection']['configuration'];teststart=time.perf_counter()
                if cfg is not None:
                    for test in ([] if args.smoke else test_cases(p,m)):
                        row=log.evaluate(cfg,test['id'],'test',c['campaign']);row['group']=test['group'];c['testRows'].append(row)
                c['testWallSeconds']=time.perf_counter()-teststart
                c['primaryTaskSuccesses']=sum(r['taskPassed'] for r in c['testRows'] if r['group']=='primary');c['boundaryTaskSuccesses']=sum(r['taskPassed'] for r in c['testRows'] if r['group']=='boundary')
                (out_dir/(c['campaign']+'.json')).write_text(json.dumps(c,indent=2,allow_nan=False)+'\n')
        finally:log.close()
        info=bridge.info
    stable=all(sha(ROOT/f)==h for f,h in hashes.items())
    summaries=[{k:v for k,v in c.items() if k not in ['trainingRows','validationRows','testRows']} for c in campaigns]
    sources={'smac_facade':inspect.getfile(AlgorithmConfigurationFacade),'smac_intensifier':inspect.getfile(Intensifier)}
    report={'schema':'cartpole-sequential-tuning/v1','mode':started['mode'],'sourceSha256':hashes,'sourcesStable':stable,'protocolSha256':PROTOCOL_HASH,
        'versions':{k:md.version(k) for k in ['smac','ConfigSpace','scikit-learn','numpy','scipy']},'upstreamSourceSha256':{k:sha(Path(v)) for k,v in sources.items()},
        'physics':info['physics'],'fit':info['fit'],'campaigns':summaries,'pairedStatistics':[] if args.smoke else paired_statistics(campaigns,p),
        'freshTestLock':lock_receipt,'totalWallSeconds':time.perf_counter()-start,'actualEvaluations':log.count,'rawReceipt':'test-results/sequential-tuning/raw-evaluations.jsonl.gz',
        'experimentValid':stable,'defaultPromoted':False,'hardware':'NOT_EVALUATED',
        'scope':['Actual SMAC3 random-forest EI with standard Intensifier. Random-racing changes only acquisition randomization to probability1; it still uses SMAC infrastructure.','Every search call is a full 600-step task or an explicit rejection/failure. No rollout-length fidelity. No cross-method cache.','Equal 108-call search cap, common initial12 calls. Certification uses fully evaluated configurations only; later validation/test calls and wall time are reported separately.','Grid216 is a finite extra reference, not a continuous-domain optimum. Grid_budget randomly orders that grid, with shared baselines first.','Training and validation are previously known development templates. Fresh test seeds are not new physics. All selections lock before any test.','Five optimizer seeds on one task: pilot statistical evidence, not a universal SOTA speed/quality claim. Failure counts and full-duration score remain separate.','Ordinal scalar feedback exactly preserves full-three-instance failure-first ranking under checked score bounds; partial racing averages are not full certificates.']}
    (ROOT/('test-results/sequential_smoke.json' if args.smoke else 'evidence/sequential_tuning.json')).write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'actualEvaluations':log.count,'summaries':[(c['campaign'],c['primaryTaskSuccesses'],c['selection'].get('score')) for c in campaigns],'statistics':report['pairedStatistics'],'valid':stable},indent=2))
    if not stable:raise RuntimeError('Source changed during evaluation')

if __name__=='__main__':main()
