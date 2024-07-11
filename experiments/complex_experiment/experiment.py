from collections import defaultdict

from shasta.base_experiment import BaseExperiment

from .custom_primitive import FormationWithPlanning
from .states import StatesExtractor
from .actions import SimpleActionDecoder
from gymnasium import spaces


def group_actors_by_type(actor_groups):
    actor_type = defaultdict(list)
    for key, value in sorted(actor_groups.items()):
        actor_type[value[0].type].extend(value)
    return actor_type


class SearchingExperiment(BaseExperiment):
    def __init__(self, config, core, experiment_config=None, *args, **kargs):
        super().__init__(config, core, experiment_config, *args, **kargs)

        self.state_extractor = StatesExtractor(experiment_config, core)
        self.actions_decoder = SimpleActionDecoder(experiment_config)

        # Primitive setup
        self.actions = {}
        env_map = core.get_map()
        for i in range(6):
            self.actions[i] = FormationWithPlanning(env_map)

    def reset(self):
        """Called at the beginning and each time the simulation is reset"""
        pass

    def get_action_space(self):
        """Returns the action space"""
        return spaces.multi_discrete.MultiDiscrete([6] * 5)

    def get_observation_space(self):
        """Returns the observation space"""
        return spaces.Box(low=0, high=1, shape=(34,))

    def get_actions(self):
        """Returns the actions"""
        pass

    def apply_actions(self, actions, core):
        """Given the action, returns a carla.VehicleControl() which will be applied to the hero

        :param action: value outputted by the policy
        to apply action we need the pareto points of each vehicle. we get that from states.py

        """
        paretos = self.state_extractor.get_pareto_node()
        decoded_action = self.actions_decoder.get_action(actions, paretos)

        # Run until the UAVs and UGVs reach the building
        while True:
            actor_groups = core.get_actor_groups()
            self.actions_done = []
            for group_id, action_args in decoded_action.items():
                self.actions_done.append(
                    self.actions[group_id].execute(actor_groups[group_id], action_args)
                )

            # Step the simulation
            core.tick()

            if all(self.actions_done):
                for group_id in actor_groups.keys():
                    self.actions[group_id].path_points = None
                break

    def get_observation(self, observation, core):
        """Function to do all the post processing of observations (sensor data).

        :param sensor_data: dictionary {sensor_name: sensor_data}

        Should return a tuple or list with two items, the processed observations,
        as well as a variable with additional information about such observation.
        The information variable can be empty
        """
        actor_groups = core.get_actor_groups()
        grouped_by_type = group_actors_by_type(actor_groups)
        states = self.state_extractor.get_state(
            grouped_by_type['uav'], grouped_by_type['ugv']
        )

        # Update progress
        done = self.state_extractor.update_progrees(
            grouped_by_type['uav'], grouped_by_type['ugv']
        )

        return states, {"searching_done": done}

    def get_truncated_status(self, observation, core):
        """Function to indicate that the episode is terminated because it reached an artificial limit set,
        for example: time or timesteps exceeded

        Should return a bool with True if the episode exceeded the limit set, False otherwise
        """
        return False


    def get_done_status(self, observation, core):
        """Returns whether the episode has ended.

        Should return True if the episode ended due to the agent achieving a goal, failing, or any other task-specific termination condition
        """
        return self.actions_done

    def compute_reward(self, observation, core):
        """Computes the reward"""
        return 0
