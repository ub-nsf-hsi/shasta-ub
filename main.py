import yaml
import time

from utils import skip_run

from shasta.env import ShastaEnv
from shasta.preprocessing.utils import extract_building_info

from experiments.simple_experiment import SimpleExperiment
from experiments.complex_experiment import SearchingExperiment
from experiments.actor_groups import create_actor_groups

from gui.gui_main import MainGUI

config_path = 'config/simulation_config.yml'
config = yaml.load(open(str(config_path)), Loader=yaml.SafeLoader)

with skip_run('skip', 'Test New Framework') as check, check():

    actor_groups = create_actor_groups()
    config['experiment']['type'] = SimpleExperiment

    env = ShastaEnv(config, actor_groups=actor_groups)

    for i in range(50000):
        env.step(0)
        time.sleep(0.01)

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

with skip_run('skip', 'Test New GUI') as check, check():
    gui = MainGUI(1200, 800, config)
    gui.run()
