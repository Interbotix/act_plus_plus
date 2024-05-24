import os

from aloha.constants import JOINT_NAMES
import cv2
import fnmatch
import h5py
import matplotlib.pyplot as plt
import numpy as np
import torch
from torch.utils.data import DataLoader
import torchvision.transforms as transforms


STATE_NAMES = JOINT_NAMES + ['gripper']


def flatten_list(list_to_flatten):
    return [item for sublist in list_to_flatten for item in sublist]


class EpisodicDataset(torch.utils.data.Dataset):
    def __init__(
        self,
        dataset_path_list,
        camera_names,
        norm_stats,
        episode_ids,
        episode_len,
        chunk_size,
        policy_class
    ):
        super(EpisodicDataset).__init__()
        self.episode_ids = episode_ids
        self.dataset_path_list = dataset_path_list
        self.camera_names = camera_names
        self.norm_stats = norm_stats
        self.episode_len = episode_len
        self.chunk_size = chunk_size
        self.cumulative_len = np.cumsum(self.episode_len)
        self.max_episode_len = max(episode_len)
        self.policy_class = policy_class
        if self.policy_class == 'Diffusion':
            self.augment_images = True
        else:
            self.augment_images = False
        self.transformations = None
        self.__getitem__(0)  # initialize self.is_sim and self.transformations
        self.is_sim = False

    # def __len__(self):
    #     return sum(self.episode_len)

    def _locate_transition(self, index):
        assert index < self.cumulative_len[-1]
        episode_index = np.argmax(self.cumulative_len > index)  # argmax returns first True index
        start_ts = index - (self.cumulative_len[episode_index] - self.episode_len[episode_index])
        episode_id = self.episode_ids[episode_index]
        return episode_id, start_ts

    def __getitem__(self, index):
        episode_id, start_ts = self._locate_transition(index)
        dataset_path = self.dataset_path_list[episode_id]
        try:
            # print(dataset_path)
            with h5py.File(dataset_path, 'r') as root:
                is_sim = root.attrs.get('sim', False)
                compressed = root.attrs.get('compress', False)
                if '/base_action' in root:
                    base_action = root['/base_action'][()]
                    base_action = preprocess_base_action(base_action)
                    action = np.concatenate([root['/action'][()], base_action], axis=-1)
                else:
                    action = root['/action'][()]
                    dummy_base_action = np.zeros([action.shape[0], 2])
                    action = np.concatenate([action, dummy_base_action], axis=-1)
                original_action_shape = action.shape
                episode_len = original_action_shape[0]
                # get observation at start_ts only
                qpos = root['/observations/qpos'][start_ts]
                # qvel = root['/observations/qvel'][start_ts]
                image_dict = dict()
                for cam_name in self.camera_names:
                    image_dict[cam_name] = root[f'/observations/images/{cam_name}'][start_ts]

                if compressed:
                    for cam_name in image_dict.keys():
                        decompressed_image = cv2.imdecode(image_dict[cam_name], 1)
                        image_dict[cam_name] = np.array(decompressed_image)

                # get all actions after and including start_ts
                if is_sim:
                    action = action[start_ts:]
                    action_len = episode_len - start_ts
                else:
                    # hack, to make timesteps more aligned
                    action = action[max(0, start_ts - 1):]
                    action_len = episode_len - max(0, start_ts - 1)

            # self.is_sim = is_sim
            padded_action = np.zeros(
                (self.max_episode_len, original_action_shape[1]),
                dtype=np.float32,
            )
            padded_action[:action_len] = action
            is_pad = np.zeros(self.max_episode_len)
            is_pad[action_len:] = 1

            padded_action = padded_action[:self.chunk_size]
            is_pad = is_pad[:self.chunk_size]

            # new axis for different cameras
            all_cam_images = []
            for cam_name in self.camera_names:
                all_cam_images.append(image_dict[cam_name])
            all_cam_images = np.stack(all_cam_images, axis=0)

            # construct observations
            image_data = torch.from_numpy(all_cam_images)
            qpos_data = torch.from_numpy(qpos).float()
            action_data = torch.from_numpy(padded_action).float()
            is_pad = torch.from_numpy(is_pad).bool()

            # channel last
            image_data = torch.einsum('k h w c -> k c h w', image_data)

            # augmentation
            if self.transformations is None:
                print('Initializing transformations')
                original_size = image_data.shape[2:]
                ratio = 0.95
                self.transformations = [
                    transforms.RandomCrop(
                        size=[int(original_size[0] * ratio), int(original_size[1] * ratio)]
                    ),
                    transforms.Resize(original_size, antialias=True),
                    transforms.RandomRotation(degrees=[-5.0, 5.0], expand=False),
                    transforms.ColorJitter(brightness=0.3, contrast=0.4, saturation=0.5),
                ]

            if self.augment_images:
                for transform in self.transformations:
                    image_data = transform(image_data)

            # normalize image and change dtype to float
            image_data = image_data / 255.0

            if self.policy_class == 'Diffusion':
                # normalize to [-1, 1]
                action_data = ((action_data - self.norm_stats["action_min"]) / (self.norm_stats["action_max"] - self.norm_stats["action_min"])) * 2 - 1
            else:
                # normalize to mean 0 std 1
                action_data = (action_data - self.norm_stats["action_mean"]) / self.norm_stats["action_std"]

            qpos_data = (qpos_data - self.norm_stats["qpos_mean"]) / self.norm_stats["qpos_std"]

        except Exception as e:
            print(f"Error loading {dataset_path} in __getitem__: '{e}'")
            quit()

        # print(image_data.dtype, qpos_data.dtype, action_data.dtype, is_pad.dtype)
        return image_data, qpos_data, action_data, is_pad


def get_norm_stats(dataset_path_list):
    all_qpos_data = []
    all_action_data = []
    all_episode_len = []

    for dataset_path in dataset_path_list:
        try:
            with h5py.File(dataset_path, 'r') as root:
                qpos = root['/observations/qpos'][()]
                # qvel = root['/observations/qvel'][()]
                if '/base_action' in root:
                    base_action = root['/base_action'][()]
                    base_action = preprocess_base_action(base_action)
                    action = np.concatenate([root['/action'][()], base_action], axis=-1)
                else:
                    action = root['/action'][()]
                    dummy_base_action = np.zeros([action.shape[0], 2])
                    action = np.concatenate([action, dummy_base_action], axis=-1)
        except Exception as e:
            print(f'Error loading {dataset_path} in get_norm_stats')
            print(e)
            quit()
        all_qpos_data.append(torch.from_numpy(qpos))
        all_action_data.append(torch.from_numpy(action))
        all_episode_len.append(len(qpos))
    all_qpos_data = torch.cat(all_qpos_data, dim=0)
    all_action_data = torch.cat(all_action_data, dim=0)

    # normalize action data
    action_mean = all_action_data.mean(dim=[0]).float()
    action_std = all_action_data.std(dim=[0]).float()
    action_std = torch.clip(action_std, 1e-2, np.inf)  # clipping

    # normalize qpos data
    qpos_mean = all_qpos_data.mean(dim=[0]).float()
    qpos_std = all_qpos_data.std(dim=[0]).float()
    qpos_std = torch.clip(qpos_std, 1e-2, np.inf)  # clipping

    action_min = all_action_data.min(dim=0).values.float()
    action_max = all_action_data.max(dim=0).values.float()

    eps = 0.0001
    stats = {
        "action_mean": action_mean.numpy(),
        "action_std": action_std.numpy(),
        "action_min": action_min.numpy() - eps,
        "action_max": action_max.numpy() + eps,
        "qpos_mean": qpos_mean.numpy(),
        "qpos_std": qpos_std.numpy(),
        "example_qpos": qpos
    }

    return stats, all_episode_len


def find_all_hdf5(dataset_dir, skip_mirrored_data):
    hdf5_files = []
    for root, dirs, files in os.walk(dataset_dir):
        for filename in fnmatch.filter(files, '*.hdf5'):
            if 'features' in filename:
                continue
            if skip_mirrored_data and 'mirror' in filename:
                continue
            hdf5_files.append(os.path.join(root, filename))
    print(f'Found {len(hdf5_files)} hdf5 files')
    return hdf5_files


def BatchSampler(batch_size, episode_len_l, sample_weights):
    sample_probs = np.array(sample_weights) / np.sum(sample_weights) if sample_weights is not None else None
    sum_dataset_len_l = np.cumsum([0] + [np.sum(episode_len) for episode_len in episode_len_l])
    while True:
        batch = []
        for _ in range(batch_size):
            episode_idx = np.random.choice(len(episode_len_l), p=sample_probs)
            step_idx = np.random.randint(sum_dataset_len_l[episode_idx], sum_dataset_len_l[episode_idx + 1])
            batch.append(step_idx)
        yield batch


def load_data(
    dataset_dir_l,
    name_filter,
    camera_names,
    batch_size_train,
    batch_size_val,
    chunk_size,
    skip_mirrored_data=False,
    policy_class=None,
    stats_dir_l=None,
    sample_weights=None,
    train_ratio=0.99
):
    if type(dataset_dir_l) == str:
        dataset_dir_l = [dataset_dir_l]
    dataset_path_list_list = [find_all_hdf5(dataset_dir, skip_mirrored_data) for dataset_dir in dataset_dir_l]
    num_episodes_0 = len(dataset_path_list_list[0])
    dataset_path_list = flatten_list(dataset_path_list_list)
    dataset_path_list = [n for n in dataset_path_list if name_filter(n)]
    num_episodes_l = [len(dataset_path_list) for dataset_path_list in dataset_path_list_list]
    num_episodes_cumsum = np.cumsum(num_episodes_l)

    # obtain train test split on dataset_dir_l[0]
    shuffled_episode_ids_0 = np.random.permutation(num_episodes_0)
    train_episode_ids_0 = shuffled_episode_ids_0[:int(train_ratio * num_episodes_0)]
    val_episode_ids_0 = shuffled_episode_ids_0[int(train_ratio * num_episodes_0):]
    train_episode_ids_l = [train_episode_ids_0] + [np.arange(num_episodes) + num_episodes_cumsum[idx] for idx, num_episodes in enumerate(num_episodes_l[1:])]
    val_episode_ids_l = [val_episode_ids_0]
    train_episode_ids = np.concatenate(train_episode_ids_l)
    val_episode_ids = np.concatenate(val_episode_ids_l)
    print(f'\n\nData from: {dataset_dir_l}\n- Train on {[len(x) for x in train_episode_ids_l]} episodes\n- Test on {[len(x) for x in val_episode_ids_l]} episodes\n\n')
    _, all_episode_len = get_norm_stats(dataset_path_list)
    train_episode_len_l = [[all_episode_len[i] for i in train_episode_ids] for train_episode_ids in train_episode_ids_l]
    val_episode_len_l = [[all_episode_len[i] for i in val_episode_ids] for val_episode_ids in val_episode_ids_l]
    train_episode_len = flatten_list(train_episode_len_l)
    val_episode_len = flatten_list(val_episode_len_l)
    if stats_dir_l is None:
        stats_dir_l = dataset_dir_l
    elif type(stats_dir_l) == str:
        stats_dir_l = [stats_dir_l]
    norm_stats, _ = get_norm_stats(flatten_list([find_all_hdf5(stats_dir, skip_mirrored_data) for stats_dir in stats_dir_l]))
    print(f'Norm stats from: {stats_dir_l}')

    batch_sampler_train = BatchSampler(
        batch_size_train,
        train_episode_len_l,
        sample_weights,
    )
    batch_sampler_val = BatchSampler(
        batch_size_val,
        val_episode_len_l,
        None,
    )

    # construct dataset and dataloader
    train_dataset = EpisodicDataset(
        dataset_path_list,
        camera_names,
        norm_stats,
        train_episode_ids,
        train_episode_len,
        chunk_size,
        policy_class,
    )
    val_dataset = EpisodicDataset(
        dataset_path_list,
        camera_names,
        norm_stats,
        val_episode_ids,
        val_episode_len,
        chunk_size,
        policy_class,
    )
    train_num_workers = 8
    val_num_workers = 8 if train_dataset.augment_images else 2
    print(f'Augment images: {train_dataset.augment_images}, train_num_workers: {train_num_workers}, val_num_workers: {val_num_workers}')
    train_dataloader = DataLoader(train_dataset, batch_sampler=batch_sampler_train, pin_memory=True, num_workers=train_num_workers, prefetch_factor=2)
    val_dataloader = DataLoader(val_dataset, batch_sampler=batch_sampler_val, pin_memory=True, num_workers=val_num_workers, prefetch_factor=2)

    return train_dataloader, val_dataloader, norm_stats, train_dataset.is_sim


def calibrate_linear_vel(base_action, c=None):
    if c is None:
        c = 0.0  # 0.19
    v = base_action[..., 0]
    w = base_action[..., 1]
    base_action = base_action.copy()
    base_action[..., 0] = v - c * w
    return base_action


def smooth_base_action(base_action):
    return np.stack([
        np.convolve(base_action[:, i], np.ones(5)/5, mode='same') for i in range(base_action.shape[1])
    ], axis=-1).astype(np.float32)


def preprocess_base_action(base_action):
    # base_action = calibrate_linear_vel(base_action)
    base_action = smooth_base_action(base_action)

    return base_action


def postprocess_base_action(base_action):
    linear_vel, angular_vel = base_action
    linear_vel *= 1.0
    angular_vel *= 1.0
    # angular_vel = 0
    # if np.abs(linear_vel) < 0.05:
    #     linear_vel = 0
    return np.array([linear_vel, angular_vel])


# helper functions

def compute_dict_mean(epoch_dicts):
    result = {k: None for k in epoch_dicts[0]}
    num_items = len(epoch_dicts)
    for k in result:
        value_sum = 0
        for epoch_dict in epoch_dicts:
            value_sum += epoch_dict[k]
        result[k] = value_sum / num_items
    return result


def detach_dict(d):
    new_d = dict()
    for k, v in d.items():
        new_d[k] = v.detach()
    return new_d


def set_seed(seed):
    torch.manual_seed(seed)
    np.random.seed(seed)


def save_videos(video, dt, video_path=None):
    if isinstance(video, list):
        cam_names = list(video[0].keys())
        cam_names = sorted(cam_names)
        h, w, _ = video[0][cam_names[0]].shape
        w = w * len(cam_names)
        fps = int(1/dt)
        out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
        for ts, image_dict in enumerate(video):
            images = []
            for cam_name in cam_names:
                image = image_dict[cam_name]
                image = image[:, :, [2, 1, 0]]  # swap B and R channel
                images.append(image)
            images = np.concatenate(images, axis=1)
            out.write(images)
        out.release()
        print(f'Saved video to: {video_path}')
    elif isinstance(video, dict):
        cam_names = list(video.keys())
        cam_names = sorted(cam_names)
        all_cam_videos = []
        for cam_name in cam_names:
            all_cam_videos.append(video[cam_name])
        all_cam_videos = np.concatenate(all_cam_videos, axis=2)  # width dimension

        n_frames, h, w, _ = all_cam_videos.shape
        fps = int(1 / dt)
        out = cv2.VideoWriter(video_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
        for t in range(n_frames):
            image = all_cam_videos[t]
            image = image[:, :, [2, 1, 0]]  # swap B and R channel
            out.write(image)
        out.release()
        print(f'Saved video to: {video_path}')


def visualize_joints(qpos_list, command_list, plot_path=None, ylim=None, label_overwrite=None):
    if label_overwrite:
        label1, label2 = label_overwrite
    else:
        label1, label2 = 'State', 'Command'

    qpos = np.array(qpos_list)  # ts, dim
    command = np.array(command_list)
    num_ts, num_dim = qpos.shape
    h, w = 2, num_dim
    num_figs = num_dim
    fig, axs = plt.subplots(num_figs, 1, figsize=(w, h * num_figs))

    # plot joint state
    all_names = [name + '_left' for name in STATE_NAMES] + [name + '_right' for name in STATE_NAMES]
    for dim_idx in range(num_dim):
        ax = axs[dim_idx]
        ax.plot(qpos[:, dim_idx], label=label1)
        ax.set_title(f'Joint {dim_idx}: {all_names[dim_idx]}')
        ax.legend()

    # plot arm command
    for dim_idx in range(num_dim):
        ax = axs[dim_idx]
        ax.plot(command[:, dim_idx], label=label2)
        ax.legend()

    if ylim:
        for dim_idx in range(num_dim):
            ax = axs[dim_idx]
            ax.set_ylim(ylim)

    plt.tight_layout()
    plt.savefig(plot_path)
    print(f'Saved qpos plot to: {plot_path}')
    plt.close()


def visualize_timestamp(t_list, dataset_path):
    plot_path = dataset_path.replace('.pkl', '_timestamp.png')
    h, w = 4, 10
    fig, axs = plt.subplots(2, 1, figsize=(w, h*2))
    # process t_list
    t_float = []
    for secs, nsecs in t_list:
        t_float.append(secs + nsecs * 10E-10)
    t_float = np.array(t_float)

    ax = axs[0]
    ax.plot(np.arange(len(t_float)), t_float)
    ax.set_title('Camera frame timestamps')
    ax.set_xlabel('timestep')
    ax.set_ylabel('time (sec)')

    ax = axs[1]
    ax.plot(np.arange(len(t_float)-1), t_float[:-1] - t_float[1:])
    ax.set_title('dt')
    ax.set_xlabel('timestep')
    ax.set_ylabel('time (sec)')

    plt.tight_layout()
    plt.savefig(plot_path)
    print(f'Saved timestamp plot to: {plot_path}')
    plt.close()


def load_hdf5(dataset_dir, dataset_name):
    dataset_path = os.path.join(dataset_dir, dataset_name + '.hdf5')
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at \n{dataset_path}\n')
        exit()

    with h5py.File(dataset_path, 'r') as root:
        qpos = root['/observations/qpos'][()]
        qvel = root['/observations/qvel'][()]
        action = root['/action'][()]
        image_dict = dict()
        for cam_name in root['/observations/images/'].keys():
            image_dict[cam_name] = root[f'/observations/images/{cam_name}'][()]

    return qpos, qvel, action, image_dict
