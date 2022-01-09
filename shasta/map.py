import warnings

from pathlib import Path

import numpy as np
import osmnx as ox
import pandas as pd
import networkx as nx

from .assets import assets_root


class Map():
    def __init__(self) -> None:
        return None

    def _setup(self, experiment_config):
        self.experiment_config = experiment_config

        # Read path for ths assets
        try:
            self.asset_path = '/'.join(
                [assets_root, self.experiment_config['map_to_use']])
        except FileNotFoundError:
            try:
                self.asset_path = '/'.join(
                    [assets_root, self.experiment_config['map_to_use']])
            except FileNotFoundError:
                raise FileNotFoundError(
                    f"Please verify the {self.experiment_config['map_to_use']} is available in asset folder"
                )

        # Initialize the assests
        self._affine_transformation_and_graph()
        self._setup_buildings()
        return None

    def get_affine_transformation_and_graph(self):
        return self.A, self.node_graph

    def _affine_transformation_and_graph(self):
        """Performs initial conversion of the lat lon to cartesian
        """
        # Graph
        read_path = self.asset_path + '/map.osm'
        G = ox.graph_from_xml(read_path, simplify=True, bidirectional='walk')
        self.node_graph = nx.convert_node_labels_to_integers(G)

        # Transformation matrix
        read_path = self.asset_path + '/coordinates.csv'
        points = pd.read_csv(read_path)
        target = points[['x', 'z']].values
        source = points[['lat', 'lon']].values

        # Pad the points with ones
        X = np.hstack((source, np.ones((source.shape[0], 1))))
        Y = np.hstack((target, np.ones((target.shape[0], 1))))
        self.A, res, rank, s = np.linalg.lstsq(X, Y, rcond=None)

        return None

    def _setup_buildings(self):
        """Perfrom initial building setup.
        """
        read_path = self.asset_path + '/buildings.csv'

        # Check if building information is already generated
        if Path(read_path).is_file():
            buildings = pd.read_csv(read_path)
        else:
            read_path = self.asset_path + '/map.osm'
            G = ox.graph_from_xml(read_path)
            # TODO: This method doesn't work if the building info is not there in OSM
            nodes, streets = ox.graph_to_gdfs(G)

            west, north, east, south = nodes.geometry.total_bounds
            polygon = ox.utils_geo.bbox_to_poly(north, south, east, west)
            gdf = ox.geometries.geometries_from_polygon(
                polygon, tags={'building': True})
            buildings_proj = ox.project_gdf(gdf)

            # Save the dataframe representing buildings
            buildings = pd.DataFrame()
            buildings['lon'] = gdf['geometry'].centroid.x
            buildings['lat'] = gdf['geometry'].centroid.y
            buildings['area'] = buildings_proj.area
            buildings['perimeter'] = buildings_proj.length
            try:
                buildings['height'] = buildings_proj['height']
            except KeyError:
                buildings['height'] = 10  # assumption
            buildings['id'] = np.arange(len(buildings_proj))

            # Save the building info
            save_path = self.asset_path + '/buildings.csv'
            buildings.to_csv(save_path, index=False)

        self.buildings = buildings
        return None

    def get_node_graph(self):
        """Get the node graph of the world

        Returns
        -------
        networkx graph
            A node graph of the world map
        """
        return self.node_graph

    def get_node_info(self, node_index):
        """Get the information about a node.

        Parameters
        ----------
        id : int
            Node ID

        Returns
        -------
        dict
            A dictionary containing all the information about the node.
        """
        return self.node_graph.nodes[node_index]

    def convert_to_lat_lon(self, point):
        """Convert a given point to lat lon co-ordinates

        Parameters
        ----------
        point : array
            A numpy array in pybullet cartesian co-ordinates

        Returns
        -------
        lat_lon : array
            The lat lon co-ordinates
        """
        point[2] = 1
        lat_lon = np.dot(point, np.linalg.inv(self.A))
        return lat_lon

    def convert_from_lat_lon(self, point):
        """Convert a lat lon co-ordinates to cartesian coordinates

        Parameters
        ----------
        point : array
            A numpy array in lat lon co-ordinates co-ordinates

        Returns
        -------
        lat_lon : array
            The cartesian coordinates
        """
        return np.dot([point[0], point[1], 1], self.A)

    def get_building_info(self, building_index):
        """Get the information about a building such as perimeter,
            position, number of floors.

            Parameters
            ----------
            id : int
                Building ID

            Returns
            -------
            dict
                A dictionary containing all the information about the building.
            """
        return self.buildings.loc[self.buildings['id'] == building_index]

    def get_lat_lon_spawn_points(self, n_points=5):

        # TODO: Verify if the projection is correct and the warning is not
        # affecting the values
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            gdf = ox.utils_geo.sample_points(self.node_graph, n_points)
            points = np.vstack([gdf.centroid.y, gdf.centroid.x]).T
        return points

    def get_catersian_node_position(self, node_index):
        node_info = self.get_node_info(node_index=node_index)
        lat = node_info['y']
        lon = node_info['x']
        cartesian_pos = np.dot([lat, lon, 1], self.A)
        return cartesian_pos

    def get_catersian_spawn_points(self, n_points=5):
        lat_lon_points = self.get_lat_lon_spawn_points(n_points)

        catersian_spawn_points = []
        for point in lat_lon_points:
            catersian_spawn_points.append(
                np.dot([point[0], point[1], 1], self.A))
        return catersian_spawn_points[0]

    def get_all_buildings(self):
        return self.buildings

    def get_transformation_matrix(self):
        return self.A
