import yaml
import time

from utils import skip_run

from shasta.env import ShastaEnv
from shasta.preprocessing.utils import extract_building_info

from experiments.simple_experiment import SimpleExperiment
from experiments.complex_experiment import SearchingExperiment
from experiments.actor_groups import create_actor_groups
from experiments.agents.uav import UaV
from stable_baselines3 import PPO

config_path = 'config/simulation_config.yml'
config = yaml.load(open(str(config_path)), Loader=yaml.SafeLoader)


#SimpleExperiment works with Gymnasium
with skip_run('run', 'Test New Framework') as check, check():
    n_actor_groups = 3
    actor_groups = {}
    for i in range(n_actor_groups): 
        actor_groups[i] = UaV()
    config['experiment']['type'] = SimpleExperiment
    env = ShastaEnv(config, actor_groups=actor_groups)
    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./SearchExperiment/", n_steps=5)
    model.learn(total_timesteps=6000, log_interval=10)
    model.save("SearchExperiment")

#SearchingExperiment works with Gym
with skip_run('skip', 'Test Experiment Framework') as check, check():
    # Create actor groups
    actor_groups = create_actor_groups()
    # Setup experiment
    exp_config_path = 'experiments/complex_experiment/complex_experiment_config.yml'
    exp_config = yaml.load(open(str(exp_config_path)), Loader=yaml.SafeLoader)
    config['experiment']['type'] = SearchingExperiment
    config['experiment']['config'] = exp_config
    env = ShastaEnv(config, actor_groups=actor_groups)
    # Check step and reset
    observation, reward, done, info = env.step([0, 0, 0, 0, 0, 0])
    env.reset()
    observation, reward, done, info = env.step([1, 1, 1, 1, 1, 1])

with skip_run('skip', 'Test Building') as check, check():
    osm_path = 'assets/buffalo-small/map.osm'
    extract_building_info(osm_path, save_fig=False)
