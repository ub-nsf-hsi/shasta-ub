import numpy as np
import networkx as nx
import osmnx as ox


class Formation(object):
    """ Formation control primitive using region based shape control.
    Coded by: Apurvakumar Jani, Date: 18/9/2019
    """
    def __init__(self):
        # Initialise the parameters
        self.a = 5
        self.b = 5
        self.knn = 6
        self.alpha = 0.5
        self.gamma = 0.5
        self.min_dis = 2
        self.dt = 1.0
        return None

    def calculate_vel(self, vehicle, dt, all_drones_pos, centroid_pos,
                      path_vel, vmax, formation_type):
        """Calculate the vehicle velocity depending on the position of the peer vehicles

        Parameters
        ----------
        vehicle : class instance
            A class instance of UxV agent
        dt : float
            Time step duration (in seconds) to use in next position calculation
        all_drones_pos : aarray
            An array with position of all the vehicles in the group/platoon
        centroid_pos : array
            An array specifying the centroid of the platoon
        path_vel : float
            Path velocity calculated from next position and current position
        vmax : float
            Maximum velocity of the vehicle
        formation_type : str
            Whether the formation is solid or ring

        Returns
        -------
        vehicle : class instance
            A vehicle class instance with updated position
        """

        # Get the neighboor position
        curr_pos = np.asarray(vehicle.current_pos[0:2])
        peers_pos = all_drones_pos

        # Calculate the velocity of each neighboor particle
        k = 1 / self.knn  # constant
        g_lij = (self.min_dis**2) - np.linalg.norm(
            curr_pos - peers_pos, axis=1, ord=2)
        del_g_ij = 2 * (peers_pos - curr_pos)
        P_ij = k * np.dot(
            np.maximum(0, g_lij / (self.min_dis**2))**2, del_g_ij)
        f_g_ij = np.linalg.norm(
            (curr_pos - centroid_pos[0:2]) / np.array([self.a, self.b]),
            ord=2) - 1

        # Calculate path velocity
        kl = 1  # constant
        del_f_g_ij = 1 * (curr_pos - centroid_pos[0:2])
        del_zeta_ij = (kl * max(0, f_g_ij)) * del_f_g_ij
        vel = path_vel - (self.alpha * del_zeta_ij) - (self.gamma * P_ij)

        # Calculate the speed
        speed = np.linalg.norm(vel)

        # Normalize the velocity with respect to speed
        if speed > vmax:
            vel = (vel / speed) * vmax

        # New position
        vehicle.desired_pos[0:2] = vehicle.current_pos[0:2] + (vel) * dt

        return vehicle, speed

    def execute(self, vehicles, next_pos, centroid_pos, formation_type):
        """Get the position of the formation control

        Parameters
        ----------
        vehicles : list
            A list containing UAV or UGV class
        centroid_pos : array
            An array containing the x, y, and z position
        dt : float
            Time step to be used for distance calculation
        """
        vmax = vehicles[0].speed
        all_drones_pos = np.asarray(
            [vehicle.current_pos[0:2] for vehicle in vehicles])

        # Path velocity
        path = np.array([next_pos[0], next_pos[1]]) - centroid_pos[0:2]
        path_vel = (1 / self.dt) * path
        if np.linalg.norm(path_vel) > vmax:
            path_vel = (path_vel / np.linalg.norm(path_vel)) * vmax

        # Loop over each drone to calculate the velocity
        # NOTE: Very complicated way to implementing list comprehension
        # TODO: Need to find an efficient way to implement formation control
        vehicles, speed = map(
            list,
            zip(*[
                self.calculate_vel(vehicle, self.dt, all_drones_pos,
                                   centroid_pos, path_vel, vmax,
                                   formation_type) for vehicle in vehicles
            ]))
        if np.max(speed) < 0.015 * len(all_drones_pos):
            formation_done = True
        else:
            formation_done = False

        # Apply the control
        for vehicle in vehicles:
            vehicle.apply_action(vehicle.desired_pos)

        return vehicles, formation_done


class PathPlanning(object):
    """Path planner based on the skeleton of the image.
    Generates a spline path
    """
    def __init__(self, env_map):
        self.map = env_map
        self.A, self.G = self.map.get_affine_transformation_and_graph()
        return None

    def linear_refine_implicit(self, x, n):
        """Given a 2D ndarray (npt, m) of npt coordinates in m dimension,
        insert 2**(n-1) additional points on each trajectory segment
        Returns an (npt*2**(n-1), m) ndarray

        Parameters
        ----------
        x : array
            A 2D input array
        n : int
            Number of intermediate points to insert between two consecutive points in x

        Returns
        -------
        array
            An array with interploated points

        Raises
        ------
        NotImplementedError
            The functions is not implemented for 3D or higher dimensions
        ValueError
            Number of intermediate points should be greated than zero
        """
        if n > 1:
            m = 0.5 * (x[:-1] + x[1:])
            if x.ndim == 2:
                msize = (x.shape[0] + m.shape[0], x.shape[1])
            else:
                raise NotImplementedError

            x_new = np.empty(msize, dtype=x.dtype)
            x_new[0::2] = x
            x_new[1::2] = m
            return self.linear_refine_implicit(x_new, n - 1)
        elif n == 1:
            return x
        else:
            raise ValueError

    def find_path(self, start, end, n_splits=1):
        """Finds a path between start and end using path graph

        Parameters
        ----------
        start : array
            A catersian co-ordinate specifying the start position
        end : array
            A node ID specifying the end position
        n_splits : int, optional
            Number of splits in refining the path points, by default 1

        Returns
        -------
        path_points : array
            A refined path points in pybullet cartesian co-ordinate system
        """
        x = []
        y = []

        # Convert the node index to node ID
        # TODO: Very dirty way to implement
        # TODO: Verify the implementation of nearest nodes
        if not isinstance(start, int):
            start_lat_lon = self.map.convert_to_lat_lon(start)
            start = ox.distance.nearest_nodes(self.G,
                                              X=start_lat_lon[1],
                                              Y=start_lat_lon[0])
        if not isinstance(end, int):
            end_lat_lon = self.map.convert_to_lat_lon(end)
            end = ox.distance.nearest_nodes(self.G,
                                            X=end_lat_lon[1],
                                            Y=end_lat_lon[0])

        route = nx.shortest_path(self.G, start, end, weight='length')
        for u, v in zip(route[:-1], route[1:]):
            # if there are parallel edges, select the shortest in length
            data = min(self.G.get_edge_data(u, v).values(),
                       key=lambda d: d["length"])
            if "geometry" in data:
                # if geometry attribute exists, add all its coords to list
                xs, ys = data["geometry"].xy
                x.extend(xs)
                y.extend(ys)
            else:
                # otherwise, the edge is a straight line from node to node
                x.extend((self.G.nodes[u]["x"], self.G.nodes[v]["x"]))
                y.extend((self.G.nodes[u]["y"], self.G.nodes[v]["y"]))

        # Add more path points for smooth travel
        lat_lon = np.array((x, y)).T
        refined_points = self.linear_refine_implicit(lat_lon, n=n_splits)
        refined_points = np.hstack(
            (refined_points, np.ones((refined_points.shape[0], 1))))

        # Exchange x and y as they are reversed in pybullet
        refined_points[:, [1, 0]] = refined_points[:, [0, 1]]

        # Convert the lat lon to pybullet co-ordinates
        path_points = np.dot(refined_points, self.A)

        return path_points

    def execute(self):
        raise NotImplementedError
