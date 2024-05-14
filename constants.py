# flake8: noqa

import os
import pathlib

### Set to True for Mobile ALOHA, False for Stationary ALOHA
IS_MOBILE = True  # for Mobile ALOHA
# IS_MOBILE = False  # for Stationary ALOHA

COLOR_IMAGE_TOPIC_NAME = '{}/color/image_rect_raw'  # for RealSense cameras
# COLOR_IMAGE_TOPIC_NAME = 'usb_{}/image_raw'  # for USB cameras

### Task parameters
DATA_DIR = os.path.expanduser('~/aloha_data')

### Fixed constants
DT = 0.02
FPS = 50
JOINT_NAMES = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
START_ARM_POSE = [0, -0.96, 1.16, 0, -0.3, 0, 0.02239, -0.02239,  0, -0.96, 1.16, 0, -0.3, 0, 0.02239, -0.02239]

XML_DIR = str(pathlib.Path(__file__).parent.resolve()) + '/assets/' # note: absolute path

LEADER_GRIPPER_CLOSE_THRESH = 0.0

# Left finger position limits (qpos[7]), right_finger = -1 * left_finger
LEADER_GRIPPER_POSITION_OPEN = 0.0323
LEADER_GRIPPER_POSITION_CLOSE = 0.0185
FOLLOWER_GRIPPER_POSITION_OPEN = 0.0579
FOLLOWER_GRIPPER_POSITION_CLOSE = 0.0440

# Gripper joint limits (qpos[6])
LEADER_GRIPPER_JOINT_OPEN = 0.8298
LEADER_GRIPPER_JOINT_CLOSE = -0.0552
FOLLOWER_GRIPPER_JOINT_OPEN = 1.6214
FOLLOWER_GRIPPER_JOINT_CLOSE = 0.6197

############################ Helper functions ############################

LEADER_GRIPPER_POSITION_NORMALIZE_FN = lambda x: (x - LEADER_GRIPPER_POSITION_CLOSE) / (LEADER_GRIPPER_POSITION_OPEN - LEADER_GRIPPER_POSITION_CLOSE)
FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN = lambda x: (x - FOLLOWER_GRIPPER_POSITION_CLOSE) / (FOLLOWER_GRIPPER_POSITION_OPEN - FOLLOWER_GRIPPER_POSITION_CLOSE)
LEADER_GRIPPER_POSITION_UNNORMALIZE_FN = lambda x: x * (LEADER_GRIPPER_POSITION_OPEN - LEADER_GRIPPER_POSITION_CLOSE) + LEADER_GRIPPER_POSITION_CLOSE
FOLLOWER_GRIPPER_POSITION_UNNORMALIZE_FN = lambda x: x * (FOLLOWER_GRIPPER_POSITION_OPEN - FOLLOWER_GRIPPER_POSITION_CLOSE) + FOLLOWER_GRIPPER_POSITION_CLOSE
LEADER2FOLLOWER_POSITION_FN = lambda x: FOLLOWER_GRIPPER_POSITION_UNNORMALIZE_FN(LEADER_GRIPPER_POSITION_NORMALIZE_FN(x))

LEADER_GRIPPER_JOINT_NORMALIZE_FN = lambda x: (x - LEADER_GRIPPER_JOINT_CLOSE) / (LEADER_GRIPPER_JOINT_OPEN - LEADER_GRIPPER_JOINT_CLOSE)
FOLLOWER_GRIPPER_JOINT_NORMALIZE_FN = lambda x: (x - FOLLOWER_GRIPPER_JOINT_CLOSE) / (FOLLOWER_GRIPPER_JOINT_OPEN - FOLLOWER_GRIPPER_JOINT_CLOSE)
LEADER_GRIPPER_JOINT_UNNORMALIZE_FN = lambda x: x * (LEADER_GRIPPER_JOINT_OPEN - LEADER_GRIPPER_JOINT_CLOSE) + LEADER_GRIPPER_JOINT_CLOSE
FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN = lambda x: x * (FOLLOWER_GRIPPER_JOINT_OPEN - FOLLOWER_GRIPPER_JOINT_CLOSE) + FOLLOWER_GRIPPER_JOINT_CLOSE
LEADER2FOLLOWER_JOINT_FN = lambda x: FOLLOWER_GRIPPER_JOINT_UNNORMALIZE_FN(LEADER_GRIPPER_JOINT_NORMALIZE_FN(x))

LEADER_GRIPPER_VELOCITY_NORMALIZE_FN = lambda x: x / (LEADER_GRIPPER_POSITION_OPEN - LEADER_GRIPPER_POSITION_CLOSE)
FOLLOWER_GRIPPER_VELOCITY_NORMALIZE_FN = lambda x: x / (FOLLOWER_GRIPPER_POSITION_OPEN - FOLLOWER_GRIPPER_POSITION_CLOSE)

LEADER_POS2JOINT = lambda x: LEADER_GRIPPER_POSITION_NORMALIZE_FN(x) * (LEADER_GRIPPER_JOINT_OPEN - LEADER_GRIPPER_JOINT_CLOSE) + LEADER_GRIPPER_JOINT_CLOSE
LEADER_JOINT2POS = lambda x: LEADER_GRIPPER_POSITION_UNNORMALIZE_FN((x - LEADER_GRIPPER_JOINT_CLOSE) / (LEADER_GRIPPER_JOINT_OPEN - LEADER_GRIPPER_JOINT_CLOSE))
FOLLOWER_POS2JOINT = lambda x: FOLLOWER_GRIPPER_POSITION_NORMALIZE_FN(x) * (FOLLOWER_GRIPPER_JOINT_OPEN - FOLLOWER_GRIPPER_JOINT_CLOSE) + FOLLOWER_GRIPPER_JOINT_CLOSE
FOLLOWER_JOINT2POS = lambda x: FOLLOWER_GRIPPER_POSITION_UNNORMALIZE_FN((x - FOLLOWER_GRIPPER_JOINT_CLOSE) / (FOLLOWER_GRIPPER_JOINT_OPEN - FOLLOWER_GRIPPER_JOINT_CLOSE))

LEADER_GRIPPER_JOINT_MID = (LEADER_GRIPPER_JOINT_OPEN + LEADER_GRIPPER_JOINT_CLOSE)/2

SIM_TASK_CONFIGS = {
    'aloha_mobile_hello_aloha':{
        'dataset_dir': DATA_DIR + '/aloha_mobile_hello_aloha',
        'episode_len': 800,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },

    'sim_transfer_cube_scripted':{
        'dataset_dir': DATA_DIR + '/sim_transfer_cube_scripted',
        'num_episodes': 50,
        'episode_len': 400,
        'camera_names': ['top', 'left_wrist', 'right_wrist']
    },

    'sim_transfer_cube_human':{
        'dataset_dir': DATA_DIR + '/sim_transfer_cube_human',
        'num_episodes': 50,
        'episode_len': 400,
        'camera_names': ['top']
    },

    'sim_insertion_scripted': {
        'dataset_dir': DATA_DIR + '/sim_insertion_scripted',
        'num_episodes': 50,
        'episode_len': 400,
        'camera_names': ['top', 'left_wrist', 'right_wrist']
    },

    'sim_insertion_human': {
        'dataset_dir': DATA_DIR + '/sim_insertion_human',
        'num_episodes': 50,
        'episode_len': 500,
        'camera_names': ['top']
    },
    'all': {
        'dataset_dir': DATA_DIR + '/',
        'num_episodes': None,
        'episode_len': None,
        'name_filter': lambda n: 'sim' not in n,
        'camera_names': ['cam_high', 'cam_left_wrist', 'cam_right_wrist']
    },

    'sim_transfer_cube_scripted_mirror':{
        'dataset_dir': DATA_DIR + '/sim_transfer_cube_scripted_mirror',
        'num_episodes': None,
        'episode_len': 400,
        'camera_names': ['top', 'left_wrist', 'right_wrist']
    },

    'sim_insertion_scripted_mirror': {
        'dataset_dir': DATA_DIR + '/sim_insertion_scripted_mirror',
        'num_episodes': None,
        'episode_len': 400,
        'camera_names': ['top', 'left_wrist', 'right_wrist']
    },

}
