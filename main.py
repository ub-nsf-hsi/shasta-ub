import yaml
import time

from utils import skip_run

from shasta.env import ShastaEnv

from experiments.simple_experiment import SimpleExperiment
from experiments.actor_groups import create_actor_groups

config_path = 'config/simulation_config.yml'
config = yaml.load(open(str(config_path)), Loader=yaml.SafeLoader)

with skip_run('run', 'Test New Framework') as check, check():

    actor_groups = create_actor_groups()
    config['experiment']['type'] = SimpleExperiment

    env = ShastaEnv(config, actor_groups=actor_groups)

    for i in range(50000):
        env.step(0)
        time.sleep(0.01)
