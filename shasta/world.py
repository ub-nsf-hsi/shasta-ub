import numpy as np

import pybullet as p
from pybullet_utils import bullet_client as bc

from .map import Map


class World():
    def __init__(self, config):
        super(World, self).__init__()
        # Need to specify some parameters
        self.config = config
        self.actor_ids = []

        # Setup the map
        self.map = Map()

        # Setup the phsysics client
        self._setup_physics_client()

        return None

    def _setup_physics_client(self):
        """Setup the physics client

        Returns
        -------
        None
        """
        # Usage mode
        if self.config['headless']:
            self.physics_client = bc.BulletClient(connection_mode=p.DIRECT)
        else:
            options = '--background_color_red=0.85 --background_color_green=0.85 --background_color_blue=0.85'  # noqa
            self.physics_client = bc.BulletClient(connection_mode=p.GUI,
                                                  options=options)

            # Set the camera parameters
            self.camer_distance = 150.0
            self.camera_yaw = 0.0
            self.camera_pitch = -89.999
            self.camera_target_position = [0, 30, 0]
            self.physics_client.resetDebugVisualizerCamera(
                cameraDistance=self.camer_distance,
                cameraYaw=self.camera_yaw,
                cameraPitch=self.camera_pitch,
                cameraTargetPosition=self.camera_target_position)

            self.physics_client.configureDebugVisualizer(
                self.physics_client.COV_ENABLE_GUI, 0)

        # Set gravity
        self.physics_client.setGravity(0, 0, -9.81)

        # Set parameters for simulation
        self.physics_client.setPhysicsEngineParameter(
            fixedTimeStep=self.config['time_step'] / 10,
            numSubSteps=1,
            numSolverIterations=5)
        return None

    def load_world_model(self, read_path):
        """Load the URDF model

        Parameters
        ----------
        read_path : str
            The path to the URDF model
        """
        self.physics_client.loadURDF(
            read_path, [0, 0, 0],
            self.physics_client.getQuaternionFromEuler([np.pi / 2, 0, 0]),
            flags=self.physics_client.URDF_USE_MATERIAL_COLORS_FROM_MTL,
            useFixedBase=True)

    def get_physics_client(self):
        """Ge the physics client

        Returns
        -------
        object
            The bullet physics client
        """
        return self.physics_client

    def get_map(self):
        """Get the map object

        Returns
        -------
        object
            The map object
        """
        return self.map

    def change_camera_position(self,
                               camera_distance=150.0,
                               camera_yaw=0.0,
                               camera_pitch=-89.999,
                               camera_target_position=[0, 30, 0]):
        """Change the world camera position

        Parameters
        ----------
        camera_distance : float, optional
            Camera distance from target, by default 150
        camera_yaw : float, optional
            Camera yaw, by default 0
        camera_pitch : float, optional
            Camera pitch, by default -89.999
        camera_target_position : list, optional
            Camera target position, by default [0, 30, 0]

        Returns
        -------
        None
        """
        self.camer_distance = camera_distance
        self.camera_yaw = camera_yaw
        self.camera_pitch = camera_pitch
        self.camera_target_position = camera_target_position

        self.physics_client.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=camera_target_position)
        return None

    def spawn_actor(self, actor, spawn_point):
        """Spawn the actor at a given point

        Parameters
        ----------
        actor : object
            An actor to spawm
        spawn_point : array
            Spawn point

        Returns
        -------
        None
        """
        # Check if the physic client is added to the actor
        if actor.physics_client is None:
            actor.physics_client = self.physics_client

        if actor.init_pos is None:
            actor.init_pos = spawn_point
        else:
            actor.init_pos = self.map.convert_from_lat_lon(actor.init_pos)

        # Load the actor
        actor._load()
        self.actor_ids.append(actor.get_actor_id())
        return None

    def tick(self):
        """Step the physics simulation
        """
        self.physics_client.stepSimulation()

    def disconnect(self):
        """Disconnect the physics client
        """
        p.disconnect(self.physics_client._client)
