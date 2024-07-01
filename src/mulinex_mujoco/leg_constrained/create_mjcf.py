import mujoco
import os

abs_path = os.path.dirname(os.path.abspath(__file__))

model = mujoco.MjModel.from_xml_path(abs_path+'/leg_constrained.urdf')
mujoco.mj_saveLastXML(abs_path+'pippo.xml', model)