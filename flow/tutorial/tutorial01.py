from flow.networks.ring import RingNetwork
# Network的参数
# * name
# * vehicles
# * net_params
# * initial_config
# * traffic_lights

name = "ring_example"

from flow.core.params import VehicleParams

vehicles = VehicleParams()
# VehicleParams，识别车辆的动态行为，是否为agent
#        initial_config 描述模拟开始时网络中车辆数量，属性
#         分为加速度，和routing两部分
from flow.controllers.car_following_models import IDMController
from flow.controllers.routing_controllers import ContinuousRouter

vehicles.add("human",
             acceleration_controller=(IDMController, {}),
             routing_controller=(ContinuousRouter, {}),
             num_vehicles=22)

# `NetParams`
from flow.networks.ring import ADDITIONAL_NET_PARAMS
print(ADDITIONAL_NET_PARAMS)

# I `ADDITIONAL_NET_PARAMS` 
# * **length**: length of the ring road
# * **lanes**: number of lanes
# * **speed**: speed limit for all edges
# * **resolution**: resolution of the curves on the ring. Setting this value to 1 converts the ring to a diamond.
from flow.core.params import NetParams
net_params = NetParams(additional_params=ADDITIONAL_NET_PARAMS)

# `InitialConfig` 仿真开始时车辆的位置
#%%
from flow.core.params import InitialConfig

initial_config = InitialConfig(spacing="uniform", perturbation=1)

#%% [markdown]
# ### 2.5 TrafficLightParams
# 
# `TrafficLightParams` are used to describe the positions and types of traffic lights in the network. These inputs are outside the scope of this tutorial, and instead are covered in `exercise06_traffic_lights.ipynb`. For our example, we create an empty `TrafficLightParams` object, thereby ensuring that none are placed on any nodes.

#%%
from flow.core.params import TrafficLightParams

traffic_lights = TrafficLightParams()

#%% [markdown]
# ## 3. Setting up an Environment
# 
# Several envionrments in Flow exist to train autonomous agents of different forms (e.g. autonomous vehicles, traffic lights) to perform a variety of different tasks. These environments are often network or task specific; however, some can be deployed on an ambiguous set of networks as well. One such environment, `AccelEnv`, may be used to train a variable number of vehicles in a fully observable network with a *static* number of vehicles.

#%%
from flow.envs.ring.accel import AccelEnv

#%% [markdown]
# Although we will not be training any autonomous agents in this exercise, the use of an environment allows us to view the cumulative reward simulation rollouts receive in the absence of autonomy.
# 
# Envrionments in Flow are parametrized by three components:
# * `EnvParams`
# * `SumoParams`
# * `Network`
# 
# ### 3.1 SumoParams
# `SumoParams` specifies simulation-specific variables. These variables include the length a simulation step (in seconds) and whether to render the GUI when running the experiment. For this example, we consider a simulation step length of 0.1s and activate the GUI.
# 
# Another useful parameter is `emission_path`, which is used to specify the path where the emissions output will be generated. They contain a lot of information about the simulation, for instance the position and speed of each car at each time step. If you do not specify any emission path, the emission file will not be generated. More on this in Section 5.

#%%
from flow.core.params import SumoParams

sumo_params = SumoParams(sim_step=0.1, render=True, emission_path='data')

#%% [markdown]
# ### 3.2 EnvParams
# 
# `EnvParams` specify environment and experiment-specific parameters that either affect the training process or the dynamics of various components within the network. Much like `NetParams`, the attributes associated with this parameter are mostly environment specific, and can be found in the environment's `ADDITIONAL_ENV_PARAMS` dictionary.

#%%
from flow.envs.ring.accel import ADDITIONAL_ENV_PARAMS

print(ADDITIONAL_ENV_PARAMS)

#%% [markdown]
# Importing the `ADDITIONAL_ENV_PARAMS` variable, we see that it consists of only one entry, "target_velocity", which is used when computing the reward function associated with the environment. We use this default value when generating the `EnvParams` object.

#%%
from flow.core.params import EnvParams

env_params = EnvParams(additional_params=ADDITIONAL_ENV_PARAMS)

#%% [markdown]
# ## 4. Setting up and Running the Experiment
# Once the inputs to the network and environment classes are ready, we are ready to set up a `Experiment` object.

#%%
from flow.core.experiment import Experiment

#%% [markdown]
# These objects may be used to simulate rollouts in the absence of reinforcement learning agents, as well as acquire behaviors and rewards that may be used as a baseline with which to compare the performance of the learning agent. In this case, we choose to run our experiment for one rollout consisting of 3000 steps (300 s).
# 
# **Note**: When executing the below code, remeber to click on the    <img style="display:inline;" src="img/play_button.png"> Play button after the GUI is rendered.

#%%
# create the network object
network = RingNetwork(name="ring_example",
                      vehicles=vehicles,
                      net_params=net_params,
                      initial_config=initial_config,
                      traffic_lights=traffic_lights)

# create the environment object
env = AccelEnv(env_params, sumo_params, network)

# create the experiment object
exp = Experiment(env)

# run the experiment for a set number of rollouts / time steps
_ = exp.run(1, 3000, convert_to_csv=True)

#%% [markdown]
# As we can see from the above simulation, the initial perturbations in the network instabilities propogate and intensify, eventually leading to the formation of stop-and-go waves after approximately 180s.
# 
# ## 5. Visualizing Post-Simulation
# 
# Once the simulation is done, a .xml file will be generated in the location of the specified `emission_path` in `SumoParams` (assuming this parameter has been specified) under the name of the network. In our case, this is:

#%%
import os

emission_location = os.path.join(exp.env.sim_params.emission_path, exp.env.network.name)
print(emission_location + '-emission.xml')

#%% [markdown]
# The .xml file contains various vehicle-specific parameters at every time step. This information is transferred to a .csv file if the `convert_to_csv` parameter in `exp.run()` is set to True. This file looks as follows:

#%%
import pandas as pd

pd.read_csv(emission_location + '-emission.csv')

#%% [markdown]
# As you can see, each row contains vehicle information for a certain vehicle (specified under the *id* column) at a certain time (specified under the *time* column). These information can then be used to plot various representations of the simulation, examples of which can be found in the `flow/visualize` folder.
#%% [markdown]
# ## 6. Modifying the Simulation
# This tutorial has walked you through running a single lane ring road experiment in Flow. As we have mentioned before, these simulations are highly parametrizable. This allows us to try different representations of the task. For example, what happens if no initial perturbations are introduced to the system of homogenous human-driven vehicles?
# 
# ```
# initial_config = InitialConfig()
# ```
# 
# In addition, how does the task change in the presence of multiple lanes where vehicles can overtake one another?
# 
# ```
# net_params = NetParams(
#     additional_params={
#         'length': 230, 
#         'lanes': 2, 
#         'speed_limit': 30, 
#         'resolution': 40
#     }
# )
# ```
# 
# Feel free to experiment with all these problems and more!
# 
# ## Bibliography
# [1] Sugiyama, Yuki, et al. "Traffic jams without bottlenecks—experimental evidence for the physical mechanism of the formation of a jam." New journal of physics 10.3 (2008): 033001.
# 
# [2] Treiber, Martin, Ansgar Hennecke, and Dirk Helbing. "Congested traffic states in empirical observations and microscopic simulations." Physical review E 62.2 (2000): 1805.

#%%


