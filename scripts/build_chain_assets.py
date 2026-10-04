"""Extend the canonical uniform-rod MJCF into a serial, unactuated N-pole chain.

Reuse reference: dm_control/suite/cartpole.py::_make_model, Apache-2.0,
commit 87e046bfeab1d6c1ffb40f9ee2a7459a38778c74. This local asset preserves
our explicit inertias/force units; it does not copy the dm_control reward or claim
its benchmark scores. Dynamics remain MuJoCo; no chain equations are derived here.
"""
from __future__ import annotations
import argparse, copy, hashlib, json, math, xml.etree.ElementTree as ET
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
MAX_POLES=32  # resource admission bound, not a physical solvability theorem

def chain_xml(poles:int, *,lengths=None,masses=None,cart_mass=1.)->str:
    if type(poles) is not int or not 1<=poles<=MAX_POLES:raise ValueError('poles must be an integer from 1 to 32 (resource bound)')
    lengths=[1.]*poles if lengths is None else list(lengths)
    masses=[.1]*poles if masses is None else list(masses)
    if len(lengths)!=poles or len(masses)!=poles:raise ValueError('One mass and full length per pole required')
    if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or v<=0 for v in [cart_mass,*lengths,*masses]):raise ValueError('Positive finite physical parameters required')
    base=(ROOT/'assets/cartpole.xml').read_text()
    if poles==1 and lengths==[1.] and masses==[.1] and cart_mass==1.:return base
    root=ET.fromstring(base);root.set('model',f'cartpole-serial-{poles}-uniform-rods')
    cart=root.find('./worldbody/body');pole=cart.find('body');template=copy.deepcopy(pole)
    cart.find('inertial').set('mass',str(cart_mass));cart.remove(pole);parent=cart
    for i,(length,mass) in enumerate(zip(lengths,masses),start=1):
        body=copy.deepcopy(template);suffix='' if i==1 else f'_{i}'
        for element in body.iter():
            if 'name' in element.attrib:element.set('name',element.get('name')+suffix)
        body.set('pos','0 0 '+str(.06 if i==1 else lengths[i-2]))
        inertia=mass*length**2/12
        inertial=body.find('inertial');inertial.set('pos',f'0 0 {length/2}');inertial.set('mass',str(mass));inertial.set('diaginertia',f'{inertia} {inertia} {min(1e-8,inertia)}')
        body.find('geom').set('fromto',f'0 0 0 0 0 {length}')
        body.find("site[@name='pole_tip"+suffix+"']").set('pos',f'0 0 {length}')
        parent.append(body);parent=body
    ET.indent(root,space='  ')
    return '<!-- Serial N-link extension: relative hinges, one cart force, non-colliding ideal uniform rods. -->\n'+ET.tostring(root,encoding='unicode')+'\n'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--poles',type=int);parser.add_argument('--output',type=Path);parser.add_argument('--check',action='store_true')
    parser.add_argument('--lengths',type=float,nargs='+');parser.add_argument('--masses',type=float,nargs='+');parser.add_argument('--cart-mass',type=float,default=1.)
    args=parser.parse_args()
    if args.poles is not None:
        if args.output is None:parser.error('--poles requires --output')
        text=chain_xml(args.poles,lengths=args.lengths,masses=args.masses,cart_mass=args.cart_mass)
        if args.check:
            if args.output.read_text()!=text:raise AssertionError('Generated asset differs')
        else:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text)
    else:
        if args.lengths or args.masses or args.output:parser.error('Custom parameters require --poles')
        for n in range(1,9):
            path=ROOT/f'assets/chains/cartpole_{n}.xml';text=chain_xml(n)
            if args.check:
                if path.read_text()!=text:raise AssertionError(f'Stale generated asset: {path}')
            else:path.parent.mkdir(exist_ok=True);path.write_text(text)
        print('Canonical chain assets 1..8 '+('verified' if args.check else 'generated'))

if __name__=='__main__':main()
