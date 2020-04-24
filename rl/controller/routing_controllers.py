import random
import numpy as np

from flow.controllers.base_routing_controller import BaseRouter


class HighwayRouter(BaseRouter):
    """The route of rl car in highway

    Extension to the Continuous Router.

    Usage
    -----
    See base class for usage example.
    """

    def choose_route(self, env):
        """See parent class."""
        vehicles = env.k.vehicle
        veh_id = self.veh_id
        veh_edge = vehicles.get_edge(veh_id)
        veh_route = vehicles.get_route(veh_id)
        veh_next_edge = env.k.network.next_edge(veh_edge,
                                                vehicles.get_lane(veh_id))
        not_an_edge = ":"
        no_next = 0

        # elif veh_route[-1] == veh_edge:
        #     random_route = random.randint(0, len(veh_next_edge) - 1)
        #     while veh_next_edge[0][0][0] == not_an_edge:
        #         veh_next_edge = env.k.network.next_edge(
        #             veh_next_edge[random_route][0],
        #             veh_next_edge[random_route][1])
        #     next_route = [veh_edge, veh_next_edge[0][0]]
        # else:
        #     next_route = None

        next_route = None
        if veh_edge in ['highway_0']:
            next_route = [veh_edge, 'highway_1']
        elif veh_edge in ['highway_1']:
            next_route = [veh_edge, 'highway_2']

        # if len(veh_next_edge) == no_next:
        #     next_route = None

        return next_route
