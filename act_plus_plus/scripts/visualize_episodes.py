import argparse
import os

from act_plus_plus.constants import DT, JOINT_NAMES
from act_plus_plus.utils import (
    save_videos,
    visualize_joints,
    load_hdf5,
)


STATE_NAMES = JOINT_NAMES + ["gripper"]

def main(args):
    dataset_dir = args['dataset_dir']
    episode_idx = args['episode_idx']
    ismirror = args['ismirror']
    if ismirror:
        dataset_name = f'mirror_episode_{episode_idx}'
    else:
        dataset_name = f'episode_{episode_idx}'

    qpos, qvel, action, image_dict = load_hdf5(dataset_dir, dataset_name)
    save_videos(image_dict, DT, video_path=os.path.join(dataset_dir, dataset_name + '_video.mp4'))
    visualize_joints(qpos, action, plot_path=os.path.join(dataset_dir, dataset_name + '_qpos.png'))

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--dataset_dir',
        action='store',
        type=str,
        help='Task dataset directory to load from.',
        required=True,
    )
    parser.add_argument(
        '--episode_idx',
        action='store',
        type=int,
        help='Episode index of task to visualize.',
        required=False,
    )
    parser.add_argument(
        '--ismirror',
        action='store_true',
        help='True if the episode is mirrored'
    )
    main(vars(parser.parse_args()))
