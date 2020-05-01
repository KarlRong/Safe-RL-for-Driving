# the TestEnv environment is used to simply simulate the network
from flow.envs import TestEnv

# the Experiment class is used for running simulations
from flow.core.experiment import Experiment

# the base network class
from flow.networks import Network

# all other imports are standard
from flow.core.params import VehicleParams
from flow.core.params import NetParams
from flow.core.params import InitialConfig
from flow.core.params import EnvParams

# create some default parameters parameters
env_params = EnvParams()
initial_config = InitialConfig()
vehicles = VehicleParams()
vehicles.add('human', num_vehicles=1)

from flow.core.params import SumoParams

sim_params = SumoParams(render=True, sim_step=1)

import os

net_params = NetParams(
    template={
    "net":"/home/rong/Safe-RL-for-Driving/traci_pedestrian_crossing/pedcrossing.net.xml",
    # features associated with the routes vehicles take
    "vtype": "/home/rong/Safe-RL-for-Driving/traci_pedestrian_crossing/pedcrossing.add.xml",

    "rou":"/home/rong/Safe-RL-for-Driving/traci_pedestrian_crossing/data/pedcrossing.rou.xml",
    },
)

# specify the edges vehicles can originate on
initial_config = InitialConfig(
    edges_distribution=["WC"]
)


# specify the routes for vehicles in the network
class TemplateNetwork(Network):

    def specify_routes(self, net_params):
        return {"-32410#3": ["-32410#3"]}

if __name__ == "__main__":
    flow_params = dict(
        exp_tag='template',
        env_name=TestEnv,
        network=TemplateNetwork,
        simulator='traci',
        sim=sim_params,
        env=env_params,
        net=net_params,
        veh=vehicles,
        initial=initial_config,
    )

    # number of time steps
    flow_params['env'].horizon = 1000
    exp = Experiment(flow_params)

    # run the sumo simulation
    _ = exp.run(1)
