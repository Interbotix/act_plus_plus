# flake8: noqa

import os
import pathlib

# Try to import ALOHA package's DATA_DIR, else default to ~/aloha_data
try:
    from aloha.constants import DATA_DIR
except ImportError:
    DATA_DIR = os.path.expanduser('~/aloha_data')

TASK_CONFIGS = {

    'aloha_mobile_hello_aloha':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_hello_aloha',
        'episode_len': 800,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },

}
