# the TestEnv environment is used to simply simulate the network
from flow.envs import TestEnv

# the Experiment class is used for running simulations
from flow.core.experiment import Experiment

# the base network class
from flow.networks import Network
from flow.envs.base import Env

# all other imports are standard
from flow.core.params import VehicleParams, SumoCarFollowingParams, SumoLaneChangeParams
from flow.controllers import IDMController
from flow.core.params import InFlows
from flow.core.params import NetParams
from flow.core.params import TrafficLightParams
from flow.core.params import InitialConfig
from flow.core.params import EnvParams
from rl.env.ped_fmt_noltlreward import FmtPedEnvNoLtlReward
from flow.controllers import IDMController, RLController, StaticLaneChanger

from gym.spaces.box import Box
import numpy as np
import collections

# create some default parameters parameters
HORIZON = 3000
# number of rollouts per training iteration
N_ROLLOUTS = 12
# number of parallel workers
N_CPUS = 6

env_params = EnvParams(
        horizon=HORIZON,
        sims_per_step=1,
        warmup_steps=0,
        additional_params={
            "max_accel": 3,
            "max_decel": -2,
            "target_velocity": 7,
            "lane_change_duration": 4,
            "num_rl": 1,
        })
initial_config = InitialConfig(edges_distribution=['highway_0'])

vehicles = VehicleParams()
vehicles.add(
    veh_id="human",
    acceleration_controller=(IDMController, {
        "noise": 0.2
    }),
    # lane_change_controller=(StaticLaneChanger, {}),
    car_following_params=SumoCarFollowingParams(
        max_speed=7,
        speed_mode="obey_safe_speed",
    ),
    lane_change_params=SumoLaneChangeParams(
        lane_change_mode=1621,
        model="SL2015",
        lc_impatience="0.1",
        lc_time_to_impatience="1.0"
    ))
vehicles.add(
    veh_id="rl",
    acceleration_controller=(RLController, {}),
    lane_change_controller=(StaticLaneChanger, {}),
    # routing_controller=(HighwayRouter, {}),
    car_following_params=SumoCarFollowingParams(
        speed_mode="obey_safe_speed",
    ),
    lane_change_params=SumoLaneChangeParams(
        lane_change_mode=256,
        model="SL2015",
        lc_impatience="0.1",
        lc_time_to_impatience="1.0"
    ),
    num_vehicles=0)

from flow.core.params import SumoParams

sim_params = SumoParams(
        sim_step=0.2,
        render=False,
        lateral_resolution=1.0,
        restart_instance=True,
    )

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
inflow.add(
    veh_type="rl",
    edge="WC",
    vehs_per_hour=100,
    depart_lane="free",
    depart_speed=5)

net_params = NetParams(
    template={
    "net":"/home/rong/Safe-RL-for-Driving/traci_pedestrian_crossing/pedcrossing.net.xml",
    # features associated with the routes vehicles take
    "vtype": "/home/rong/Safe-RL-for-Driving/traci_pedestrian_crossing/pedcrossing.add.xml",
    # 和下方specify_routes一致
    "rou":"/home/rong/Safe-RL-for-Driving/traci_pedestrian_crossing/data/pedcrossing.rou.xml",
    "trip":"/home/rong/Safe-RL-for-Driving/traci_pedestrian_crossing/pedestrians.trip.xml"
    },
    inflows=inflow,
)

# specify the edges vehicles can originate on
initial_config = InitialConfig(
    edges_distribution=["WC"]
)

tl_logic = TrafficLightParams(baseline=False)
phases = [{"duration": "100000", "state": "GGGGr"},
          {"duration": "4", "state": "yyyyr"},
          {"duration": "10", "state": "rrrrG"},
          {"duration": "10", "state": "rrrrr"}]
tl_logic.add("C", phases=phases, programID="custom", offset="0")


# specify the routes for vehicles in the network
class PedCrossing(Network):

    def specify_routes(self, net_params):
        return {'EC': ['EC', 'CW'],
                'WC': ['WC', 'CE']}


flow_params = dict(
    exp_tag='ped_movexy',
    env_name=FmtPedEnvNoLtlReward,
    network=PedCrossing,
    simulator='traci',
    sim=sim_params,
    env=env_params,
    net=net_params,
    veh=vehicles,
    initial=initial_config,
    tls=tl_logic,
)

