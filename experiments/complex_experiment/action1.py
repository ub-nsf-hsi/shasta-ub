# -*- coding: utf-8 -*-
"""
Created on Thu Jun  9 14:18:18 2022

@author: Prajit
"""

import math as mt


class ActionDecoder:
    def __init__(self, config):
        self.config = config
        return None

    def get_action(self, net_output, pareto_nodes):
        """
        
        # 1. Node to visit for each squads - 3*3 = 9
        # 2. Size of each UAV/UGV squad - we can remove this and make the squads constant
        # 3. Degree of Avoidance for each squad - 3*3 = 9 - here there are no adversaries, so we can make it constant
        Parameters
        ----------
        net_output : continuous - gym - spaces.Box(low=0, high=1., shape=(9, )) - changed to discreet action [ nodes to be visited is discrete between 0 and 5]
            DESCRIPTION.
        pareto_nodes : dict of all pareto nodes for each vehicle

        Returns
        -------
        action : dictionary () 
            nodes to be visited by all vehicles

        """
        all_action = dict()
        for i in range(len(net_output)):
            all_action[i]  = pareto_nodes[net_output[i]]
        return all_action

    def get_idle_vehicles(self, vehicles):
        """Returns non idle vehicles

        Parameters
        ----------
        vehicles : list
            A list of UAV or UGV vehilces class
        """
        vehicles = list(filter(lambda vehicle: vehicle.idle, vehicles))
        return vehicles

