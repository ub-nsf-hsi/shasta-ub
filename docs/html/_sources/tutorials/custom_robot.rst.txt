Creating Custom Robots
======================

Introduction
------------
In this tutorial, we will learn how to create custom robot instances using the provided `BaseActor` abstract class and an example custom actor class named `UaV`. The `BaseActor` class provides a foundation for defining behaviors and properties common to different types of robotic actors within a simulation environment using the PyBullet physics engine.

Abstract Actor Class: BaseActor
-------------------------------
The `BaseActor` abstract class serves as the base for implementing specific robot instances. It defines essential methods and properties required for interacting with the simulation environment.

.. code-block:: python

    class BaseActor(with_metaclass(ABCMeta, object)):
        """This is the base class for a single UGV robot"""

        physics_client = None
        _loaded = False

        def __init__(self):
            self.states = {}
            self._actor_id = None
            self.init_pos = None
            self.init_orientation = None

        def _load(self):
            """Load object into PyBullet and return a list of loaded body ids."""
            if self._loaded:
                raise ValueError("Cannot load an actor multiple times.")
            self._loaded = True
            actor_ids = self.load_asset()

            if not isinstance(actor_ids, list):
                actor_ids = [actor_ids]

            self._actor_id = actor_ids[0]
            return actor_ids

        def get_actor_id(self):
            """Get the actor id

            Returns
            -------
            int
                The actor id in the simulation
            """
            return self._actor_id

        @abstractmethod
        def load_asset(self, *args, **kwargs):
            """Load the assets of the actor."""
            raise NotImplementedError

        @abstractmethod
        def reset(self, *args, **kwargs):
            """Reset the actor

            Raises
            ------
            NotImplementedError
            """
            raise NotImplementedError

        @abstractmethod
        def get_observation(self, *args, **kwargs):
            """Get the observation from the actor.

            Raises
            ------
            NotImplementedError
            """
            raise NotImplementedError

        @abstractmethod
        def apply_action(self, *args, **kwargs):
            """Apply the action to the actor

            Raises
            ------
            NotImplementedError
            """
            raise NotImplementedError

        @abstractmethod
        def destroy(self, *args, **kwargs):
            """Destroy the actor

            Raises
            ------
            NotImplementedError
            """
            raise NotImplementedError


Explanation of Abstract Actor Class
-------------------------------------
1. `__init__(self)`: Initializes the actor instance with default values for its properties.
2. `_load(self)`: Abstract method to be implemented by subclasses for loading objects into the PyBullet simulation.
3. `get_actor_id(self)`: Returns the unique identifier assigned to the actor in the simulation.
4. `load_asset(self)`: Abstract method to be implemented by subclasses for loading specific assets of the actor into the simulation environment.
5. `reset(self)`: Abstract method to be implemented by subclasses for resetting the actor to its initial state.
6. `get_observation(self)`: Abstract method to be implemented by subclasses for retrieving observations from the actor.
7. `apply_action(self)`: Abstract method to be implemented by subclasses for applying actions to the actor.
8. `destroy(self)`: Abstract method to be implemented by subclasses for removing the actor from the simulation environment.

Custom Actor Class: UaV
-----------------------
The `UaV` class extends the `BaseActor` class to define a custom Unmanned Aerial Vehicle (UAV) actor within the simulation environment.

.. code-block:: python

    class UaV(BaseActor):
        """This is the base class for a single UAV robot"""

        def __init__(self, config=None):
            # Platoon properties
            self.vehicle_id = 0
            self.platoon_id = 0
            self.type = 'uav'

            # Properties UAV
            self.init_pos = None
            self.init_orientation = None
            self.current_pos = copy.deepcopy(self.init_pos)
            self.desired_pos = copy.deepcopy(self.init_pos)

            # Extra parameters
            self.idle = True
            self.ammo = 100
            self.battery = 100
            self.functional = True
            self.speed = 2.5
            self.search_speed = 0.25

            return None

        def load_asset(self):
            """Initial step of objects and constraints"""
            # Initial pos and orientation
            if self.init_orientation is None:
                self.init_orientation = self.physics_client.getQuaternionFromEuler(
                    [0, 0, np.pi / 2]
                )

            # Load the mesh
            path = '/'.join(['./assets', 'vehicles', 'arial_vehicle_abstract.urdf'])
            self.object = self.physics_client.loadURDF(
                path,
                self.init_pos,
                self.init_orientation,
                flags=self.physics_client.URDF_USE_MATERIAL_COLORS_FROM_MTL,
            )
            # Constraint
            self.constraint = self.physics_client.createConstraint(
                self.object,
                -1,
                -1,
                -1,
                self.physics_client.JOINT_FIXED,
                [0, 0, 0],
                [0, 0, 0],
                self.init_pos,
            )

            # Change color depending on team type
            self.physics_client.changeVisualShape(self.object, -1, rgbaColor=[0, 0, 1, 1])
            return None

        def get_pos_and_orientation(self):
            """Returns the position and orientation (as Yaw angle) of the robot."""
            pos, rot = self.physics_client.getBasePositionAndOrientation(self.object)
            euler = self.physics_client.getEulerFromQuaternion(rot)
            return np.array(pos), euler

        def reset(self):
            """Moves the robot back to its initial position"""
            self.physics_client.changeConstraint(self.constraint, self.init_pos)
            self.current_pos = copy.deepcopy(self.init_pos)
            self.desired_pos = copy.deepcopy(self.init_pos)
            return None

        def get_observation(self):
            pos, orientation = self.get_pos_and_orientation()
            return pos

        def apply_action(self, position):
            """This function moves the vehicles to given position

            Parameters
            ----------
            position : array
                The position to which the vehicle should be moved.
            """
            self.current_pos, _ = self.get_pos_and_orientation()
            position[2] = 10.0
            self.physics_client.changeConstraint(self.constraint, position)
            return None

        def destroy(self):
            self.physics_client.removeBody(self.object)

Explanation of Custom Actor Class
---------------------------------
1. `__init__(self, config=None)`: Initializes the UAV actor instance with default properties and parameters.
2. `load_asset(self)`: Loads the UAV mesh and initializes constraints within the PyBullet simulation environment.
3. `get_pos_and_orientation(self)`: Retrieves the current position and orientation of the UAV actor from the simulation environment.
4. `reset(self)`: Resets the UAV actor to its initial position within the simulation.
5. `get_observation(self)`: Retrieves observation data from the UAV actor, typically used for state estimation or perception tasks.
6. `apply_action(self, position)`: Applies a specified action to the UAV actor, moving it to the given position within the simulation environment.
7. `destroy(self)`: Removes the UAV actor from the simulation environment.

Conclusion
----------
In this tutorial, we have learned how to create custom robot instances using the provided `BaseActor` abstract class and the example custom actor class `UaV`. By extending the `BaseActor` class and implementing specific methods, we can define custom behaviors and properties for robotic actors within a PyBullet simulation environment.