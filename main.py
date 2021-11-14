import yaml

from shasta.enhance_env import EnhanceEnv
from shasta.default_actions.default_actions import (blue_team_actions,
                                                    red_team_actions)

from shasta.visualize import plot_nodes

from utils import skip_run

config_path = 'config/simulation_config.yml'
config = yaml.load(open(str(config_path)), Loader=yaml.SafeLoader)

with skip_run('run', 'Test New Framework') as check, check():

    default_blue_actions = blue_team_actions(config)
    default_red_actions = red_team_actions(config)

    if config['simulation']['show_nodes']:
        plot_nodes(config)
    else:
        env = EnhanceEnv(config)
        env.step(default_blue_actions, default_red_actions)
