import yaml
import time

from utils import skip_run

from shasta.env import ShastaEnv
from shasta.actor import TestUxV

config_path = 'config/simulation_config.yml'
config = yaml.load(open(str(config_path)), Loader=yaml.SafeLoader)

with skip_run('run', 'Test New Framework') as check, check():

    test = TestUxV()
    actors_group = {'1': [test]}

    env = ShastaEnv(config, actors_group=actors_group)

    for i in range(50000):
        env.step(0)
        time.sleep(0.1)
