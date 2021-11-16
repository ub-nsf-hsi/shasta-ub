import os
import psutil
import signal

from .world import World

from .utils import get_initial_positions


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
    def __init__(self, config, actor_groups: dict = None):
        """Initialize the server and client"""
        self.config = config
        self.world = World(config)
        self.actor_groups = actor_groups
        self.map = self.world.get_map()

        self.init_server()

    def init_server(self):
        """Start a server on a random port"""
        pass

    def setup_experiment(self, experiment_config):
        """Initialize the hero and sensors"""

        # Load the environment
        read_path = self.map.asset_path + '/environment_collision_free.urdf'
        self.world.load_world_model(read_path)

        # Spawn the actors in thes physics client
        self.spawn_actors()

    def get_world(self):
        return self.world

    def reset(self):
        """This function resets / spawns the hero vehicle and its sensors"""

        # Reset all the actors
        observations = {}
        for group_id in self.actor_groups:
            # Check if the entry is a list or not
            if not isinstance(self.actor_groups[group_id], list):
                self.actor_groups[group_id] = [self.actor_groups[group_id]]

            obs_from_each_actor = []
            for actor in self.actor_groups[group_id]:
                # Reset the actor and collect the observation
                actor.reset()
                obs_from_each_actor.append(actor.get_observation)

            observations[group_id] = obs_from_each_actor

        return observations

    def spawn_actors(self):
        """Spawns vehicles and walkers, also setting up the Traffic Manager and its parameters"""
        for group_id in self.actor_groups:
            # Check if the entry is a list or not
            if not isinstance(self.actor_groups[group_id], list):
                self.actor_groups[group_id] = [self.actor_groups[group_id]]

            # Spawn the actors
            spawn_point = self.map.get_catersian_spawn_points()
            positions = get_initial_positions(spawn_point, 10,
                                              len(self.actor_groups[group_id]))
            for actor, position in zip(self.actor_groups[group_id], positions):
                self.world.spawn_actor(actor, position)

    def get_actor_groups(self):
        return self.actor_groups

    def get_actors_by_group_id(self, group_id):
        return self.actor_groups[group_id]

    def tick(self):
        """Performs one tick of the simulation, moving all actors, and getting the sensor data"""
        observations = {}

        # Tick once the simulation
        self.world.tick()

        # Collect the raw observation from all the actors in each actor group
        for group_id in self.actor_groups:
            obs_from_each_actor = []
            for actor in self.actor_groups[group_id]:
                obs_from_each_actor.append(actor.get_observation)

            observations[group_id] = obs_from_each_actor

        return observations
