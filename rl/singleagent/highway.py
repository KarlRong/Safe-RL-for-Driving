"""Open merge example.

Trains a a small percentage of rl vehicles to dissipate shockwaves caused by
on-ramp merge to a single lane open highway network.
"""
from flow.core.params import SumoParams, EnvParams, InitialConfig
from flow.core.params import NetParams, InFlows, SumoCarFollowingParams, SumoLaneChangeParams
# from flow.networks.merge import ADDITIONAL_NET_PARAMS
from flow.networks.highway_ramps import ADDITIONAL_NET_PARAMS
from flow.core.params import VehicleParams
from flow.controllers import IDMController, RLController
from flow.envs import MergePOEnv
# from flow.networks import MergeNetwork
# from flow.networks.highway import HighwayNetwork, ADDITIONAL_NET_PARAMS
from flow.networks import HighwayRampsNetwork


# experiment number
# - 0: 10% RL penetration,  5 max controllable vehicles
# - 1: 25% RL penetration, 13 max controllable vehicles
# - 2: 33% RL penetration, 17 max controllable vehicles
EXP_NUM = 1

# time horizon of a single rollout
HORIZON = 3000
# number of rollouts per training iteration
N_ROLLOUTS = 20
# number of parallel workers
N_CPUS = 1

# inflow rate at the highway
FLOW_RATE = 2000
# percent of autonomous vehicles
RL_PENETRATION = [0.1, 0.25, 0.33][EXP_NUM]
# num_rl term (see ADDITIONAL_ENV_PARAMs)
NUM_RL = [5, 13, 17][EXP_NUM]

# We consider a highway network with an upstream merging lane producing
# shockwaves
additional_net_params = ADDITIONAL_NET_PARAMS.copy()
# additional_net_params["merge_lanes"] = 1
# additional_net_params["highway_lanes"] = 3
# additional_net_params["pre_merge_length"] = 500

# lengths
additional_net_params["highway_length"] = 1200
additional_net_params["on_ramps_length"] = 200
additional_net_params["off_ramps_length"] = 200

# number of lanes
additional_net_params["highway_lanes"] = 3
additional_net_params["on_ramps_lanes"] = 1
additional_net_params["off_ramps_lanes"] = 1

# speed limits
additional_net_params["highway_speed"] = 30
additional_net_params["on_ramps_speed"] = 20
additional_net_params["off_ramps_speed"] = 20

# ramps
additional_net_params["on_ramps_pos"] = [400]
additional_net_params["off_ramps_pos"] = [800]

# probability of exiting at the next off-ramp
additional_net_params["next_off_ramp_proba"] = 0.25

# RL vehicles constitute 5% of the total number of vehicles
vehicles = VehicleParams()
vehicles.add(
    veh_id="human",
    acceleration_controller=(IDMController, {
        "noise": 0.2
    }),
    car_following_params=SumoCarFollowingParams(
        speed_mode="obey_safe_speed",
    ),
    lane_change_params=SumoLaneChangeParams(
        lane_change_mode=1621,
        model="SL2015",
        lc_impatience="0.1",
        lc_time_to_impatience="1.0"
    ),
    num_vehicles=5)
vehicles.add(
    veh_id="rl",
    acceleration_controller=(RLController, {}),
    car_following_params=SumoCarFollowingParams(
        speed_mode="obey_safe_speed",
    ),
    lane_change_params=SumoLaneChangeParams(
        lane_change_mode=1621,
        model="SL2015",
        lc_impatience="0.1",
        lc_time_to_impatience="1.0"
    ),
    num_vehicles=0)

# Vehicles are introduced from both sides of merge, with RL vehicles entering
# from the highway portion as well
inflow = InFlows()
inflow.add(
    veh_type="human",
    edge="highway_0",
    vehs_per_hour=(1 - RL_PENETRATION) * FLOW_RATE,
    depart_lane="free",
    depart_speed=10)
inflow.add(
    veh_type="rl",
    edge="highway_0",
    vehs_per_hour=RL_PENETRATION * FLOW_RATE,
    depart_lane="free",
    depart_speed=10)
inflow.add(
    veh_type="human",
    edge="on_ramp_0",
    vehs_per_hour=350,
    depart_lane="free",
    depart_speed=7.5)

flow_params = dict(
    # name of the experiment
    exp_tag="stabilizing_open_network_merges",

    # name of the flow environment the experiment is running on
    env_name=MergePOEnv,

    # name of the network class the experiment is running on
    # network=MergeNetwork,
    # network=HighwayNetwork,
    network=HighwayRampsNetwork,

    # simulator that is used by the experiment
    simulator='traci',

    # sumo-related parameters (see flow.core.params.SumoParams)
    sim=SumoParams(
        sim_step=0.2,
        render=True,
        lateral_resolution=1.0,
        restart_instance=True,
    ),

    # environment related parameters (see flow.core.params.EnvParams)
    env=EnvParams(
        horizon=HORIZON,
        sims_per_step=5,
        warmup_steps=0,
        additional_params={
            "max_accel": 1.5,
            "max_decel": 1.5,
            "target_velocity": 20,
            "num_rl": NUM_RL,
        },
    ),

    # network-related parameters (see flow.core.params.NetParams and the
    # network's documentation or ADDITIONAL_NET_PARAMS component)
    net=NetParams(
        inflows=inflow,
        additional_params=additional_net_params,
    ),

    # vehicles to be placed in the network at the start of a rollout (see
    # flow.core.params.VehicleParams)
    veh=vehicles,

    # parameters specifying the positioning of vehicles upon initialization/
    # reset (see flow.core.params.InitialConfig)
    initial=InitialConfig(edges_distribution=['highway_0']),
)
