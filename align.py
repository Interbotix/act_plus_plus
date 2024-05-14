from aloha.aloha.robot_utils import move_arms, torque_on
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np

# for calibrating head cam and arms being symmetrical

def main():
    global_node = create_interbotix_global_node()

    follower_bot_left = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
        robot_name='follower_left',
        node=global_node,
    )
    follower_bot_right = InterbotixManipulatorXS(
        robot_model='vx300s',
        group_name='arm',
        gripper_name='gripper',
        robot_name='follower_right',
        node=global_node,
    )

    robot_startup(global_node)

    all_bots = [follower_bot_left, follower_bot_right]
    for bot in all_bots:
        torque_on(bot)

    multiplier = np.array([-1, 1, 1, -1, 1, 1])
    follower_sleep_position_left = np.array([-0.8, -0.5, 0.5, 0, 0.65, 0])
    follower_sleep_position_right = follower_sleep_position_left * multiplier
    all_positions = [follower_sleep_position_left, follower_sleep_position_right]
    move_arms(all_bots, all_positions, move_time=2.0)

    robot_shutdown(global_node)

if __name__ == '__main__':
    main()
