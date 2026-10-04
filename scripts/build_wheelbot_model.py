"""Deterministically generate a planar primitive-only Upkie-derived MJCF.

Dimensions and component masses marked source-derived come from the pinned
Upkie description. Links are analytic solid cylinders; motor assemblies and
the tire mesh are omitted. This is a teaching simplification, not an Upkie twin.
"""
from pathlib import Path
import hashlib, math, json
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'assets/wheelbot/wheelbot.xml'

def model_xml():
    # Read and pin the source values rather than duplicating them as claims.
    source=ROOT/'assets/wheelbot/source/upkie.urdf'
    expected='d15965215067276203599a850e5c7ebb319ed6815506f0a2721dae78abac2483'
    if hashlib.sha256(source.read_bytes()).hexdigest()!=expected:
        raise ValueError('Pinned Upkie source hash mismatch')
    urdf=ET.parse(source).getroot()
    def mass(link): return float(urdf.find(f"./link[@name='{link}']/inertial/mass").get('value'))
    torso=urdf.find("./link[@name='torso']/visual/geometry/box")
    torso_dims=[float(x) for x in torso.get('size').split()]
    rod_geom=urdf.find("./link[@name='left_femur']/visual/geometry/cylinder")
    length=float(rod_geom.get('length')); r=float(rod_geom.get('radius')); rod_mass=mass('left_femur')
    tire_geom=urdf.find("./link[@name='left_wheel_tire']/collision/geometry/cylinder")
    tire_r=float(tire_geom.get('radius')); tire_w=float(tire_geom.get('length')); tire_m=mass('left_wheel_tire')
    torso_mass=mass('torso')
    rod_ix = rod_mass * (3*r*r + length*length) / 12
    rod_iz = rod_mass * r*r / 2
    tire_iy = .5*tire_m*tire_r*tire_r
    tire_ix = tire_m*(3*tire_r*tire_r+tire_w*tire_w)/12
    sx,sy,sz=torso_dims
    torso_ix=torso_mass*(sy*sy+sz*sz)/12
    torso_iy=torso_mass*(sx*sx+sz*sz)/12
    torso_iz=torso_mass*(sx*sx+sy*sy)/12
    limits=[float(urdf.find(f"./joint[@name='left_{name}']/limit").get('effort')) for name in ['hip','knee','wheel']]
    xml = f'''<mujoco model="planar-wheelbot-upkie-derived">
  <compiler angle="radian" inertiafromgeom="false"/>
  <option timestep="0.002" integrator="Euler" gravity="0 0 -9.81" cone="elliptic" impratio="10"/>
  <size njmax="1000" nconmax="100"/>
  <default>
    <joint damping="0" armature="0" limited="false"/>
    <geom condim="3" friction="1 0.01 0.001" solref="0.01 1" solimp="0.95 0.99 0.001"/>
    <motor ctrllimited="true"/>
  </default>
  <worldbody>
    <light pos="0 -2 3" dir="0 1 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" pos="0 0 0" size="5 2 0.1" rgba="0.25 0.28 0.31 1"/>
    <body name="torso" pos="0 0 0">
          <joint name="x" type="slide" axis="1 0 0"/>
          <joint name="z" type="slide" axis="0 0 1"/>
          <joint name="pitch" type="hinge" axis="0 1 0"/>
          <inertial pos="0 0 {sz/2}" mass="{torso_mass}" diaginertia="{torso_ix} {torso_iy} {torso_iz}"/>
          <geom name="torso_visual" type="box" pos="0 0 {sz/2}" size="{sx/2} {sy/2} {sz/2}" mass="0" rgba="0.92 0.94 0.97 1" contype="0" conaffinity="0"/>
          <site name="hip_site" pos="0 0 0" size="0.012" rgba="1 0.1 0.1 1"/>
          <body name="upper_link" pos="0 0 0">
          <joint name="hip" type="hinge" axis="0 1 0" damping="0.015" armature="0.0002" range="-1.26 1.26" limited="true"/>
            <inertial pos="0 0 {-length/2}" mass="{rod_mass}" diaginertia="{rod_ix} {rod_ix} {rod_iz}"/>
            <geom name="upper_link_visual" type="cylinder" pos="0 0 {-length/2}" size="{r} {length/2}" mass="0" rgba="0.18 0.42 0.88 1" contype="0" conaffinity="0"/>
            <body name="lower_link" pos="0 0 {-length}">
              <joint name="knee" type="hinge" axis="0 1 0" damping="0.015" armature="0.0002" range="-2.51 2.51" limited="true"/>
              <inertial pos="0 0 {-length/2}" mass="{rod_mass}" diaginertia="{rod_ix} {rod_ix} {rod_iz}"/>
              <geom name="lower_link_visual" type="cylinder" pos="0 0 {-length/2}" size="{r} {length/2}" mass="0" rgba="0.18 0.42 0.88 1" contype="0" conaffinity="0"/>
              <site name="knee_site" pos="0 0 0" size="0.01" rgba="1 0.1 0.1 1"/>
              <body name="wheel_body" pos="0 0 {-length}">
                <joint name="wheel" type="hinge" axis="0 1 0" damping="0.015" armature="0.0002"/>
                <inertial pos="0 0 0" mass="{tire_m}" diaginertia="{tire_ix} {tire_iy} {tire_ix}"/>
                <geom name="wheel_visual" type="cylinder" size="{tire_r} {tire_w/2}" quat="0.7071067811865476 0.7071067811865475 0 0" mass="0" rgba="0.12 0.72 0.34 1"/>
                <site name="wheel_site" pos="0 0 0" size="0.008" rgba="0.1 0.9 0.3 1"/>
              </body>
            </body>
          </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="hip_motor" joint="hip" gear="1" ctrlrange="{-limits[0]} {limits[0]}"/>
    <motor name="knee_motor" joint="knee" gear="1" ctrlrange="{-limits[1]} {limits[1]}"/>
    <motor name="wheel_motor" joint="wheel" gear="1" ctrlrange="{-limits[2]} {limits[2]}"/>
  </actuator>
  <!-- Declared display home only; solved qref/uref in profile is dynamic authority. -->
  <keyframe><key name="home" qpos="0 {tire_r+2*length*math.cos(.15)} 0 .15 -.3 0"/></keyframe>
</mujoco>
'''
    root=ET.fromstring(xml); ET.indent(root, space='  ')
    return ET.tostring(root, encoding='unicode')+'\n'

if __name__ == '__main__':
    text=model_xml()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text)
    print(OUT)
