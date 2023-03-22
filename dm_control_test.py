#Testing DM control package

from dm_control import mujoco
from dm_control import viewer
from dm_control import suite
import numpy as np
import math as m

X0 = np.array([0,0,2,0]) #x, xdot, theta (rad), thetadot (rad/s)
env = suite.load(domain_name="cartpole", task_name="balance", task_kwargs={'random': X0})

action_spec = env.action_spec()
U_nom = np.array([-0.560521, -0.377548, -0.252297, -0.167165, -0.109778, -0.0714117, -0.0459458, -0.0291897, -0.0185076, -0.0127209])

# Define a uniform random policy.
def optimum_policy(time_step):
    
    print(time_step)
    del time_step  # Unused.
    

    return U_nom[0]


viewer.launch(env, policy=optimum_policy)

"""
physics = mujoco.Physics.from_xml_string("/models/cartpole.xml")

pixels = physics.render()

with physics.reset_context():
  physics.named.data.qpos['up_down'] = 0.5

"""

