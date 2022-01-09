#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from __future__ import print_function

import gym

from .core import ShastaCore


class ShastaEnv(gym.Env):
    """
    This is a shasta environment, responsible of handling all the SHASTA related steps of the training.
    """
    def __init__(self, config, actor_groups: dict = None):
        """Initializes the environment"""
        self.config = config

        # Check if experiment config is present
        if not self.config["experiment"]:
            raise Exception("The config should have experiment configuration")

        # Setup the core
        self.core = ShastaCore(self.config, actor_groups=actor_groups)
        self.core.setup_experiment(self.config["experiment"])

        self.experiment = self.config["experiment"]["type"](
            self.config["experiment"], self.core)

        if not self.experiment:
            raise Exception(
                "The experiment type cannot be empty. Please provide an experiment class"
            )

        self.action_space = self.experiment.get_action_space()
        self.observation_space = self.experiment.get_observation_space()

        self.reset()

    def reset(self):
        self.experiment.reset()

        # Tick once and get the observations
        raw_data = self.core.reset()
        observation, _ = self.experiment.get_observation(raw_data, self.core)

        return observation

    def step(self, action):
        """Computes one tick of the environment in order to return the new observation,
        as well as the rewards"""

        self.experiment.apply_actions(action, self.core)
        raw_data = self.core.tick()

        observation, info = self.experiment.get_observation(
            raw_data, self.core)
        done = self.experiment.get_done_status(observation, self.core)
        reward = self.experiment.compute_reward(observation, self.core)

        return observation, reward, done, info
