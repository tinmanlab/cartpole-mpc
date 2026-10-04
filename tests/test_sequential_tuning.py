"""Contracts for actual SMAC/ConfigSpace, ordinal loss and existing WASM evaluation."""
import sys,json,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from sequential_tuning import load_protocol,configuration_space,grid_configurations,initial_configurations,configuration_key,encode_loss,choose_final,PlantBridge,make_smac
from ConfigSpace import Configuration
from smac.runhistory.dataclasses import TrialInfo,TrialValue

class TuningContracts(unittest.TestCase):
 def setUp(self):self.p,self.m=load_protocol()
 def test_conditional_space_and_grid(self):
  cs=configuration_space(self.p,17)
  for c in cs.sample_configuration(100):
   d=dict(c);self.assertEqual('horizon' in d,d['controller']=='hard_mpc')
   self.assertTrue(.5<=d['effort']<=2 and .1<=d['process']<=10)
  grid=grid_configurations(self.p);self.assertEqual(len(grid),72);self.assertEqual(len({configuration_key(c) for c in grid}),72)
  with self.assertRaises(ValueError):Configuration(cs,values={'controller':'lqr','observer':'kf','effort':1.,'process':1.,'horizon':30})
 def test_ordinal_loss_preserves_three_instance_priority(self):
  good={'outcome':'completed','taskPassed':True,'fullScore':.3}
  task={'outcome':'completed','taskPassed':False,'fullScore':0.}
  fail={'outcome':'solver-rejected','taskPassed':False,'fullScore':None}
  worst_good={**good,'fullScore':512.}
  self.assertGreater(encode_loss(task,self.p),3*encode_loss(worst_good,self.p))
  self.assertGreater(encode_loss(fail,self.p),3*encode_loss({**task,'fullScore':512.},self.p))
  self.assertAlmostEqual(encode_loss(good,self.p),.3/512)
  with self.assertRaises(ValueError):encode_loss({**good,'fullScore':513.},self.p)
  with self.assertRaises(ValueError):encode_loss({**fail,'outcome':'execution-error'},self.p)
 def test_partial_or_test_data_cannot_win_validation(self):
  configs=initial_configurations(self.p);rows=[]
  for config in configs:
   for t in self.m['validation']:rows.append({'configuration':config,'caseId':t['id'],'phase':'validation','outcome':'completed','taskPassed':True,'fullScore':1.})
  chosen=choose_final(rows,self.m);self.assertEqual(chosen['configuration']['controller'],'lqr');self.assertEqual(chosen['configuration']['observer'],'kf')
  self.assertIsNone(choose_final(rows[:1],self.m)['configuration'])
  with self.assertRaises(ValueError):choose_final([{**r,'phase':'test'} for r in rows],self.m)
 def test_real_smac_ask_tell_and_intensifier(self):
  with tempfile.TemporaryDirectory(dir=ROOT/'test-results') as directory:
   optimizer=make_smac(self.p,self.m,17,Path(directory),'smac_racing')
   for _ in range(16):
    info=optimizer.ask();self.assertIn(info.instance,[t['id'] for t in self.m['training']])
    d=dict(info.config);cost=(float(d['effort'])-1)**2+.1*(float(d['process'])-1)**2
    optimizer.tell(info,TrialValue(cost=cost,time=0),save=False)
   self.assertEqual(optimizer.runhistory.finished,16)
   self.assertEqual(type(optimizer.intensifier).__name__,'Intensifier')
 def test_actual_wasm_and_lock(self):
  with PlantBridge(ROOT) as bridge:
   self.assertEqual(bridge.info['physics']['backend'],'mujoco-wasm')
   c=initial_configurations(self.p)[0]
   with self.assertRaises(RuntimeError):bridge.evaluate(c,'fresh-0-18101','test')
   row=bridge.evaluate(c,self.m['training'][0]['id'],'training')
   self.assertEqual(row['appliedSteps'],600);self.assertEqual(row['information']['controller'],'observer-output only')
   bridge.lock([c])
   with self.assertRaises(RuntimeError):bridge.evaluate(c,self.m['training'][0]['id'],'training')
   result=bridge.evaluate(c,'fresh-0-18101','test');self.assertEqual(result['phase'],'test')

if __name__=='__main__':unittest.main()
