import os
import psutil
import signal

from .world import World


def kill_all_servers():
    """Kill all PIDs that start with Carla"""
    processes = [
        p for p in psutil.process_iter() if "carla" in p.name().lower()
    ]
    for process in processes:
        os.kill(process.pid, signal.SIGKILL)


class ShastaCore():
    """
    Class responsible of handling all the different CARLA functionalities, such as server-client connecting,
    actor spawning and getting the sensors data.
    """
    def __init__(self, config, actors_group: dict = None):
        """Initialize the server and client"""
        self.config = config
        self.world = World(config)
        self.actors_group = actors_group
        self.map = self.world.get_map()

        self.init_server()

    def init_server(self):
        """Start a server on a random port"""
        pass

    def setup_experiment(self, experiment_config):
        """Initialize the hero and sensors"""

        # Load the environment
        if experiment_config['detailed_model']:
            read_path = '/'.join([
                self.config['urdf_data_path'], experiment_config['map_to_use'],
                'environment_collision_free.urdf'
            ])
        else:
            read_path = '/'.join([
                self.config['urdf_data_path'], experiment_config['map_to_use'],
                'environment_collision_free.urdf'
            ])

        self.world.load_world_model(read_path)

        # Spawn the actors in thes physics client
        self.spawn_actors()

    def get_world(self):
        return self.world

    def reset(self):
        """This function resets / spawns the hero vehicle and its sensors"""
        return None

    def spawn_actors(self):
        """Spawns vehicles and walkers, also setting up the Traffic Manager and its parameters"""
        for group_id in self.actors_group:
            # Check if the entry is a list or not
            if not isinstance(self.actors_group[group_id], list):
                self.actors_group[group_id] = [self.actors_group[group_id]]

            spawn_point = self.map.get_catersian_spawn_points()
            for actor in self.actors_group[group_id]:
                self.world.spawn_actor(actor, spawn_point)

    def get_actors_group(self):
        return self.actors_group

    def get_actor_by_group_id(self, group_id):
        return self.actors_group[group_id]

    def tick(self):
        """Performs one tick of the simulation, moving all actors, and getting the sensor data"""
        observations = {}

        # Tick once the simulation
        self.world.tick()

        # Collect the raw observation from all the actors in each actor group
        for group_id in self.actors_group:
            obs_from_each_actor = []
            for actor in self.actors_group[group_id]:
                obs_from_each_actor.append(actor.get_observation)

            observations[group_id] = obs_from_each_actor

        return observations
