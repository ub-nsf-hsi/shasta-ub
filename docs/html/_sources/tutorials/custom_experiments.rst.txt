Creating Custom Experiment
==========================

This tutorial will guide you through the process of creating a custom experiment using the `BaseExperiment` class provided. We'll explain the implementation of each function required to customize the experiment and provide an example of a custom experiment class called `SearchingExperiment`.

BaseExperiment Class Overview
------------------------------

The `BaseExperiment` class provides a basic structure for defining experiments in a simulation environment. It is designed to be inherited by custom experiment classes. Here's a brief overview of the functions defined in the `BaseExperiment` class:

.. code-block:: python


    class BaseExperiment(with_metaclass(ABCMeta, object)):
        def __init__(self, config, core, experiment_config=None, *args, **kargs):
            """The base experiment which other experiments should inherit

            Parameters
            ----------
            config : yaml
                The experiment configuration file
            core : object
                The SHASTA core object

            Returns
            -------
            None
            """
            self.config = config
            self.core = core
            self.exp_config = experiment_config

            return None

        def reset(self):
            """Called at the beginning and each time the simulation is reset"""
            pass

        @abstractmethod
        def get_action_space(self):
            """Returns the action space"""
            raise NotImplementedError

        @abstractmethod
        def get_observation_space(self):
            """Returns the observation space"""
            raise NotImplementedError

        def get_actions(self):
            """Returns the actions"""
            raise NotImplementedError

        @abstractmethod
        def apply_actions(self, actions, core):
            """Given the action, applies the action to group or individual actor

            :param action: value outputted by the policy
            """
            raise NotImplementedError

        @abstractmethod
        def get_observation(self, observation, core):
            """Function to do all the post processing of observations (sensor data).

            :param sensor_data: dictionary {sensor_name: sensor_data}

            Should return a tuple or list with two items, the processed observations,
            as well as a variable with additional information about such observation.
            The information variable can be empty
            """
            return NotImplementedError

        def get_done_status(self, observation, core):
            """Returns whether or not the experiment has to end"""
            return NotImplementedError

        def compute_reward(self, observation, core):
            """Computes the reward"""
            return NotImplementedError

1. `__init__(self, config, core, experiment_config=None, *args, **kargs)`: Initializes the base experiment class with configuration parameters.
2. `reset(self)`: Called at the beginning and each time the simulation is reset.
3. `get_action_space(self)`: Returns the action space.
4. `get_observation_space(self)`: Returns the observation space.
5. `get_actions(self)`: Returns the actions.
6. `apply_actions(self, actions, core)`: Given the action, applies the action to group or individual actor.
7. `get_observation(self, observation, core)`: Processes observation data.
8. `get_done_status(self, observation, core)`: Returns whether or not the experiment has to end.
9. `compute_reward(self, observation, core)`: Computes the reward.

Custom Experiment: SearchingExperiment
---------------------------------------

The `SearchingExperiment` class is an example of a custom experiment inheriting from the `BaseExperiment` class. This experiment is designed for a simulation environment where agents (UAVs and UGVs) need to search for a target building. Below is the implementation of the `SearchingExperiment` class along with explanations for each overridden function:

.. code-block:: python

    from collections import defaultdict
    from shasta.base_experiment import BaseExperiment
    from .actions import SimpleActionDecoder
    from .custom_primitive import FormationWithPlanning
    from .states import StatesExtractor

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
            self.actions = {}
            env_map = core.get_map()
            for i in range(6):
                self.actions[i] = FormationWithPlanning(env_map)

        def reset(self):
            pass

        def get_action_space(self):
            pass

        def get_observation_space(self):
            pass

        def get_actions(self):
            pass

        def apply_actions(self, actions, core):
            paretos = self.state_extractor.get_pareto_node()
            decoded_action = self.actions_decoder.get_action(actions, paretos)
            while True:
                actor_groups = core.get_actor_groups()
                self.actions_done = []
                for group_id, action_args in decoded_action.items():
                    self.actions_done.append(
                        self.actions[group_id].execute(actor_groups[group_id], action_args)
                    )
                core.tick()
                if all(self.actions_done):
                    for group_id in actor_groups.keys():
                        self.actions[group_id].path_points = None
                    break

        def get_observation(self, observation, core):
            actor_groups = core.get_actor_groups()
            grouped_by_type = group_actors_by_type(actor_groups)
            states = self.state_extractor.get_state(
                grouped_by_type['uav'], grouped_by_type['ugv']
            )
            done = self.state_extractor.update_progrees(
                grouped_by_type['uav'], grouped_by_type['ugv']
            )
            return states, {"searching_done": done}

        def get_done_status(self, observation, core):
            return self.actions_done

        def compute_reward(self, observation, core):
            return 0

Now let's explain each function in detail:

1. `__init__(self, config, core, experiment_config=None, *args, **kargs)`: Initializes the `SearchingExperiment` class by setting up state extractors, action decoders, and primitive actions.

2. `reset(self)`: This function is called at the beginning and each time the simulation is reset. In this example, it doesn't have any implementation.

3. `get_action_space(self)`: Returns the action space. It is not implemented in this example.

4. `get_observation_space(self)`: Returns the observation space. It is not implemented in this example.

5. `get_actions(self)`: Returns the actions. It is not implemented in this example.

6. `apply_actions(self, actions, core)`: Applies the decoded actions to the actors in the simulation environment. It iterates through decoded actions, applies them to respective actor groups, and updates the simulation.

7. `get_observation(self, observation, core)`: Processes observation data. It extracts states of UAVs and UGVs from the simulation environment using the state extractor.

8. `get_done_status(self, observation, core)`: Returns whether or not the experiment has to end based on the completion status of actions.

9. `compute_reward(self, observation, core)`: Computes the reward. In this example, it returns a constant reward of 0.

This concludes the tutorial on creating a custom experiment using the `BaseExperiment` class. You can further extend and customize this experiment class according to your specific simulation requirements.
