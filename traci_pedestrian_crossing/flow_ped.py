# the TestEnv environment is used to simply simulate the network
from flow.envs import TestEnv

# the Experiment class is used for running simulations
from flow.core.experiment import Experiment

# the base network class
from flow.networks import Network

# all other imports are standard
from flow.core.params import VehicleParams, SumoCarFollowingParams, SumoLaneChangeParams
from flow.controllers import IDMController
from flow.core.params import InFlows
from flow.core.params import NetParams
from flow.core.params import InitialConfig
from flow.core.params import EnvParams

# create some default parameters parameters
env_params = EnvParams()
initial_config = InitialConfig()
vehicles = VehicleParams()
vehicles.add(
    veh_id="human",
    acceleration_controller=(IDMController, {
        "noise": 0.2
    }),
    # lane_change_controller=(StaticLaneChanger, {}),
    car_following_params=SumoCarFollowingParams(
        speed_mode="obey_safe_speed",
    ),
    lane_change_params=SumoLaneChangeParams(
        lane_change_mode=1621,
        model="SL2015",
        lc_impatience="0.1",
        lc_time_to_impatience="1.0"
    ))

from flow.core.params import SumoParams

sim_params = SumoParams(render=True, sim_step=1)

import os

inflow = InFlows()
inflow.add(veh_type="human",
           edge="WC",
           # depart_lane="best",
           depart_lane=1,
           arrivalLane=0,
           probability=0.1,
           depart_speed="random",
           )
inflow.add(veh_type="human",
           edge="WC",
           # depart_lane="best",
           depart_lane=0,
           arrivalLane=1,
           probability=0.1,
           depart_speed="random",
           )
inflow.add(veh_type="human",
           edge="EC",
           # depart_lane="best",
           # vehs_per_hour=2000,
           depart_lane=1,
           arrivalLane=0,
           probability=0.1,
           depart_speed="random",
           )
inflow.add(veh_type="human",
           edge="EC",
           # depart_lane="best",
           # vehs_per_hour=2000,
           depart_lane=0,
           arrivalLane=1,
           probability=0.1,
           depart_speed="random",
           )

net_params = NetParams(
    template={
    "net":"/home/rong/Safe-RL-for-Driving/traci_pedestrian_crossing/pedcrossing.net.xml",
    # features associated with the routes vehicles take
    "vtype": "/home/rong/Safe-RL-for-Driving/traci_pedestrian_crossing/pedcrossing.add.xml",

    # "rou":"/home/rong/Safe-RL-for-Driving/traci_pedestrian_crossing/data/pedcrossing.rou.xml",
    },
    inflows=inflow,
)

# specify the edges vehicles can originate on
initial_config = InitialConfig(
    edges_distribution=["WC"]
)


# specify the routes for vehicles in the network
class PedCrossing(Network):

    def specify_routes(self, net_params):
        return {'EC': ['EC', 'CW'],
                'WC': ['WC', 'CE']}

if __name__ == "__main__":
    flow_params = dict(
        exp_tag='template',
        env_name=TestEnv,
        network=PedCrossing,
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
