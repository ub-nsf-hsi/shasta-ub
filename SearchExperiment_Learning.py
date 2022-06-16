# -*- coding: utf-8 -*-
"""
Created on June 14 10:40:49 2022

@author: prajit
"""

import yaml
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from shasta.env import ShastaEnv
from experiments.complex_experiment import SearchingExperiment
from experiments.actor_groups import create_actor_groups

#kmeans warning 
import warnings
warnings.filterwarnings('ignore')

config_path = 'config/simulation_config.yml'
config = yaml.load(open(str(config_path)), Loader=yaml.SafeLoader)
actor_groups = create_actor_groups()

# Setup experiment
exp_config_path = 'experiments/complex_experiment/complex_experiment_config.yml'
exp_config = yaml.load(open(str(exp_config_path)), Loader=yaml.SafeLoader)
config['experiment']['type'] = SearchingExperiment
config['experiment']['config'] = exp_config

env = ShastaEnv(config, actor_groups=actor_groups)

env = DummyVecEnv([lambda: env])

#callback to save models at mentioned frequency
checkpoint_callback = CheckpointCallback(save_freq=10000, save_path='./logs/',
                                         name_prefix='rl_model')


#Learning for PPO
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./SearchExperiment/")
model.learn(total_timesteps=100000, callback=checkpoint_callback)
model.save("SearchExperiment")


