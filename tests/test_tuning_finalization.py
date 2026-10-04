"""F1 regression: a rejected design is a completed result, not a failed experiment.
Uses actual CLI finalizers and the real Node/WASM bridge. Only campaign search
outcomes are synthetic, so no tuning or physical test rollout is performed.
Run in the existing optional tuning environment after npm ci.
"""
from pathlib import Path
from contextlib import ExitStack
from unittest.mock import patch
import copy,gzip,json,sys,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import sequential_tuning as seq
import run_tuning_campaign as batch
REAL_BRIDGE=seq.PlantBridge

class FinalizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p,cls.m=seq.load_protocol()

    def make_campaign(self,method,seed):
        rows=[]
        for config in seq.initial_configurations(self.p):
            for case in self.m['validation']:
                rows.append({'configuration':config,'caseId':case['id'],'phase':'validation','outcome':'completed','taskPassed':False,'fullScore':1.0})
        choice=seq.choose_final(rows,self.m)
        self.assertIsNone(choice['configuration'])
        return {'campaign':f'{method}-{seed}','method':method,'optimizerSeed':seed,
                'selection':choice,'trainingRows':[],'validationRows':[],'testRows':[],
                'searchWallSeconds':1.0,'validationWallSeconds':0.0}

    def isolated_root(self,root):
        (root/'evidence').mkdir()
        (root/'test-results').mkdir()
        for name in batch.FILES:
            path=root/name;path.parent.mkdir(parents=True,exist_ok=True)
            path.write_bytes((ROOT/name).read_bytes())

    def assert_no_result(self,root):
        path=root/'evidence/sequential_tuning.json';r=json.loads(path.read_text())
        self.assertTrue(r['experimentValid'])
        self.assertEqual(r['actualTestEvaluations'],0)
        self.assertEqual(len(r['campaigns']),21)
        for c in r['campaigns']:
            self.assertIsNone(c['selection']['configuration'])
            self.assertEqual(c['testStatus'],'not-evaluated-no-admissible')
            self.assertEqual(c['actualTestEvaluations'],0)
            for key in ['primaryTaskSuccesses','boundaryTaskSuccesses','primaryMeanTaskScore']:
                self.assertIsNone(c[key],key)
        for s in r['pairedStatistics']:
            self.assertEqual(s['pairs'],[])
            self.assertIsNone(s['meanRelativeTaskDifference'])
            self.assertIsNone(s['conditionalPairedBootstrap95'])
        return path.read_bytes()

    def test_empty_lock_is_terminal_and_denies_all_evaluation(self):
        with REAL_BRIDGE(ROOT) as b:
            result=b.lock([])
            self.assertTrue(result['locked']);self.assertEqual(result['configurations'],0)
            config=seq.initial_configurations(self.p)[0]
            for phase,case in [('test',seq.test_cases(self.p,self.m)[0]),('training',self.m['training'][0]),('validation',self.m['validation'][0])]:
                with self.assertRaises(RuntimeError):b.evaluate(config,case['id'],phase)
            with self.assertRaises(RuntimeError):b.lock([config])
            self.assertEqual(b.call({'op':'info'})['actualCalls'],0)

    def test_bounded_no_candidate_completes_and_retry_does_no_work(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);self.isolated_root(root);out=root/'test-results/sequential-tuning';out.mkdir()
            for method,seed in [(method,seed) for seed in self.p['optimizerSeeds'] for method in self.p['methods']]+[('grid_exhaustive',self.p['optimizerSeeds'][0])]:
                c=self.make_campaign(method,seed);(out/(c['campaign']+'.json')).write_text(json.dumps(c))
                with gzip.open(out/('raw-'+c['campaign']+'.jsonl.gz'),'wt'):pass
            with ExitStack() as stack:
                stack.enter_context(patch.object(batch,'ROOT',root))
                stack.enter_context(patch.object(batch,'load_protocol',return_value=(copy.deepcopy(self.p),copy.deepcopy(self.m))))
                factory=stack.enter_context(patch.object(batch,'PlantBridge',side_effect=lambda _root:REAL_BRIDGE(ROOT)))
                stack.enter_context(patch.object(sys,'argv',['run_tuning_campaign.py','--finalize']))
                batch.main();before=self.assert_no_result(root);self.assertEqual(factory.call_count,1)
                factory.side_effect=AssertionError('Completed retry must not reopen the bridge')
                batch.main();self.assertEqual(self.assert_no_result(root),before)
                # Changed output is not accepted as a valid completion marker.
                path=root/'evidence/sequential_tuning.json';r=json.loads(path.read_text());r['actualTestEvaluations']=1;path.write_text(json.dumps(r))
                with self.assertRaisesRegex(RuntimeError,'changed|mismatch|invalid|reconcile'):batch.main()

    def test_monolithic_no_candidate_completes_and_retry_does_no_work(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td);self.isolated_root(root)
            with ExitStack() as stack:
                stack.enter_context(patch.object(seq,'ROOT',root))
                stack.enter_context(patch.object(seq,'load_protocol',return_value=(copy.deepcopy(self.p),copy.deepcopy(self.m))))
                factory=stack.enter_context(patch.object(seq,'PlantBridge',side_effect=lambda _root:REAL_BRIDGE(ROOT)))
                search=stack.enter_context(patch.object(seq,'do_campaign',side_effect=lambda p,m,method,seed,*args,**kwargs:self.make_campaign(method,seed)))
                stack.enter_context(patch.object(sys,'argv',['sequential_tuning.py']))
                seq.main();before=self.assert_no_result(root);self.assertEqual(search.call_count,21)
                factory.side_effect=AssertionError('Completed retry must not open a bridge')
                search.side_effect=AssertionError('Completed retry must not repeat search')
                seq.main();self.assertEqual(self.assert_no_result(root),before)
                self.assertEqual(factory.call_count,1);self.assertEqual(search.call_count,21)

if __name__=='__main__':unittest.main()
