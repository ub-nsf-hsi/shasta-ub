import numpy as np

from shasta.actor import BaseActor


class UgV(BaseActor):
    """This the base class for single UGV robot
    """
    def __init__(self):
        # Platoon properties
        self.vehicle_id = 0
        self.platoon_id = 0

        # Properties UAV
        self.init_pos = None
        self.init_orientation = None
        self.current_pos = self.init_pos
        self.desired_pos = self.init_pos

        # Extra parameters
        self.idle = True
        self.ammo = 100
        self.battery = 100
        self.functional = True
        self.speed = 0.5
        self.search_speed = 0.25

        return None

    def _load(self):
        """Initial step of objects and constraints
        """
        # Load the mesh
        path = '/'.join(
            ['shasta/assets', 'vehicles', 'ground_vehicle_abstract.urdf'])
        self.object = self.physics_client.loadURDF(
            path,
            self.init_pos,
            self.init_orientation,
            flags=self.physics_client.URDF_USE_MATERIAL_COLORS_FROM_MTL,
        )
        # Constraint
        self.constraint = self.physics_client.createConstraint(
            self.object, -1, -1, -1, self.physics_client.JOINT_FIXED,
            [0, 0, 0], [0, 0, 0], self.init_pos)

        # Change color depending on team type
        self.physics_client.changeVisualShape(self.object,
                                              -1,
                                              rgbaColor=[1, 0, 0, 1])
        return None

    def get_pos_and_orientation(self):
        """Returns the position and orientation (as Yaw angle) of the robot.
        """
        pos, rot = self.physics_client.getBasePositionAndOrientation(
            self.object)
        euler = self.physics_client.getEulerFromQuaternion(rot)
        return np.array(pos), euler

    def reset(self):
        """Moves the robot back to its initial position
        """
        self.physics_client.changeConstraint(self.constraint, self.init_pos)
        self.current_pos = self.init_pos
        self.desired_pos = self.init_pos
        return None

    def get_observation(self):
        return 0

    def apply_action(self, position):
        """This function moves the vehicles to given position

        Parameters
        ----------
        position : array
            The position to which the vehicle should be moved.
        """
        self.current_pos, _ = self.get_pos_and_orientation()
        position[2] = 0.5
        self.physics_client.changeConstraint(self.constraint, position)
        return None

    def destroy(self):
        self.physics_client.removeBody(self.object)
