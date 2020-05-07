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
from flow.core.params import TrafficLightParams
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


class PedCrossingEnv(TestEnv):

    def __init__(self, env_params, sim_params, network, simulator='traci'):
        super().__init__(env_params, sim_params, network, simulator)
        self.activeRequest = False
        self.greenTimeSoFar = 0
        # minimum green time for the vehicles
        self.MIN_GREEN_TIME = 15
        # the first phase in tls plan. see 'pedcrossing.tll.xml'
        self.VEHICLE_GREEN_PHASE = 0
        self.PEDESTRIAN_GREEN_PHASE = 2
        # the id of the traffic light (there is only one). This is identical to the
        # id of the controlled intersection (by default)
        self.TLSID = 'C'
        # pedestrian edges at the controlled intersection
        self.WALKINGAREAS = [':C_w0', ':C_w1']
        self.CROSSINGS = [':C_c0']

    def additional_command(self):
        # decide wether there is a waiting pedestrian and switch if the green
        # phase for the vehicles exceeds its minimum duration
        if not self.activeRequest:
            self.activeRequest = self.checkWaitingPersons()
        if self.k.kernel_api.trafficlight.getPhase(self.TLSID) == self.VEHICLE_GREEN_PHASE:
            self.greenTimeSoFar += 1
            if self.greenTimeSoFar > self.MIN_GREEN_TIME:
                # check whether someone has pushed the button

                if self.activeRequest:
                    # switch to the next phase
                    self.k.kernel_api.trafficlight.setPhase(
                        self.TLSID, self.VEHICLE_GREEN_PHASE + 1)
                    # reset state
                    self.activeRequest = False
        return

    def checkWaitingPersons(self):
        """check whether a person has requested to cross the street"""

        # check both sides of the crossing
        for edge in self.WALKINGAREAS:
            peds = self.k.kernel_api.edge.getLastStepPersonIDs(edge)
            # check who is waiting at the crossing
            # we assume that pedestrians push the button upon
            # standing still for 1s
            for ped in peds:
                if (self.k.kernel_api.person.getWaitingTime(ped) == 1 and
                        self.k.kernel_api.person.getNextEdge(ped) in self.CROSSINGS):
                    numWaiting = self.k.kernel_api.trafficlight.getServedPersonCount(self.TLSID, self.PEDESTRIAN_GREEN_PHASE)
                    print("%s: pedestrian %s pushes the button (waiting: %s)" %
                          (self.k.kernel_api.simulation.getTime(), ped, numWaiting))
                    return True
        return False


if __name__ == "__main__":
    flow_params = dict(
        exp_tag='template',
        env_name=PedCrossingEnv,
        network=PedCrossing,
        simulator='traci',
        sim=sim_params,
        env=env_params,
        net=net_params,
        veh=vehicles,
        initial=initial_config,
        tls=tl_logic,
    )

    # number of time steps
    flow_params['env'].horizon = 10000
    exp = Experiment(flow_params)

    # run the sumo simulation
    _ = exp.run(1)
