"""N-link asset contracts, using native MuJoCo rather than a second dynamics model."""
import sys, unittest, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from build_chain_assets import chain_xml

class ChainModelTests(unittest.TestCase):
    def test_one_link_is_exact_canonical_asset(self):
        self.assertEqual(chain_xml(1),(ROOT/'assets/cartpole.xml').read_text())
    def test_dimensions_serial_parentage_and_single_force(self):
        for n in range(1,33):
            xml=chain_xml(n);m=mujoco.MjModel.from_xml_string(xml)
            self.assertEqual((m.nq,m.nv,m.nu,m.na),(n+1,n+1,1,0))
            self.assertEqual(m.actuator_trnid[0,0],0)
            # world -> cart -> first pole -> each successive pole
            for body in range(2,n+2):self.assertEqual(m.body_parentid[body],body-1)
            self.assertEqual(len(ET.fromstring(xml).findall('.//actuator/motor')),1)
    def test_invalid_n_and_parameters_are_rejected(self):
        for n in [0,-1,1.5,True,33]:
            with self.assertRaises(ValueError):chain_xml(n)
        for params in [{'masses':[.1]},{'lengths':[1,-1]},{'cart_mass':0},{'masses':[.1,float('nan')]}]:
            with self.assertRaises(ValueError):chain_xml(2,**params)
    def test_relative_angles_accumulate_in_world_geometry(self):
        lengths=[.4,.6,.8];m=mujoco.MjModel.from_xml_string(chain_xml(3,lengths=lengths,masses=[.1,.2,.15]));d=mujoco.MjData(m)
        d.qpos[:]=[.2,.1,-.2,.3];mujoco.mj_forward(m,d)
        point=np.array([.2,0,.18]);angle=0.
        for i,length in enumerate(lengths):
            angle+=d.qpos[i+1];point+=length*np.array([np.sin(angle),0,np.cos(angle)])
            name='pole_tip' if i==0 else f'pole_tip_{i+1}'
            site=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,name)
            np.testing.assert_allclose(d.site_xpos[site],point,atol=1e-12)
            self.assertAlmostEqual(m.body_inertia[i+2,1],m.body_mass[i+2]*length**2/12)
    def test_unforced_upright_is_equilibrium_not_balancing_proof(self):
        for n in [2,3,4,8]:
            m=mujoco.MjModel.from_xml_string(chain_xml(n));d=mujoco.MjData(m)
            mujoco.mj_forward(m,d);np.testing.assert_allclose(d.qacc,0,atol=1e-12)

if __name__=='__main__':unittest.main()
