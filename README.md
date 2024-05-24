# Imitation Learning Algorithms and Co-training for Mobile ALOHA

Original Project Websites:

* [ALOHA](https://tonyzhaozh.github.io/aloha/)
* [Mobile ALOHA](https://mobile-aloha.github.io/)

This repo contains the implementation of ACT, Diffusion Policy, and VINN.
To work with real hardware, you would need to install [ALOHA](https://github.com/Interbotix/aloha).

# Repo Structure

* act_plus_plus
  * act_plus_plus
    * ``detr`` Model definitions of ACT, modified from [DETR](https://github.com/facebookresearch/detr)
    * ``constants.py`` Constants shared across files
    * ``policy.py`` An adaptor for ACT policy
    * ``utils.py`` Utils such as data loading and helper functions
    * scripts
      * ``imitate_episodes.py`` Train and Evaluate ACT
      * ``visualize_episodes.py`` Save videos from a .hdf5 dataset

# Installation

There are two recommended ways to install ACT: using ``conda`` or ``venv``.
Using ``venv`` is preferred due its ease of use against frameworks like ROS.

## Installation Using venv

```bash
sudo apt-get install python3-venv
python3 -m venv ~/act # creates a venv "act" in the home directory, can be created anywhere
source ~/act/bin/activate
pip install dm_control==1.0.14
pip install einops
pip install h5py
pip install ipython
pip install matplotlib
pip install modern_robotics
pip install mujoco==2.3.7
pip install opencv-python
pip install packaging
pip install pexpect
pip install pyquaternion
pip install pyyaml
pip install rospkg
pip install torch
pip install torchvision
pip install transforms3d
pip install wandb
cd /path/to/act/detr && pip install -e .
```

The ``r2d2`` branch of robomimic must also be installed.

```bash
git clone -b r2d2 https://github.com/ARISE-Initiative/robomimic.git
cd robomimic
pip install -e .
```
