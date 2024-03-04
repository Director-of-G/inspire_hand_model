#!/usr/bin/env python
"""
    Run mylaunch_hand.launch first, and control the robot
"""

import rospy

import numpy as np

import actionlib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


FINGER_UPPER_LIMITS = [0.0089, 0.00875, 0.0083, 0.0079, 0.006, 0.0042]
FINGER_LOWER_LIMITS = [-0.001, -0.001, -0.0015, -0.002, -0.0027, -0.0058]

# index in /joint_state
finger_to_index_map = {
    "0": [0, 1, 2, 3, 4],
    "6": [5, 6, 7, 8, 9],
    "7": [10, 11, 12, 13, 14],
    "8": [15, 16, 17, 18, 19],
    "4-0": [20, 21, 22],
    "4-1": [23, 24, 25, 26, 27, 28, 29],
}

finger_to_actuated_index_map = {
    "0": 0,
    "6": 1,
    "7": 2,
    "8": 3,
    "4-0": 4,
    "4-1": 5
}

finger_to_limits_map = {
    "0": (-0.001, 0.0089),
    "6": (-0.001, 0.00875),
    "7": (-0.0015, 0.0083),
    "8": (-0.002, 0.0079),
    "4-0": (-0.0027, 0.006),
    "4-1": (-0.0058, 0.0042)
}


def move_hand_to_position(joint_pos):
    print("desired joint pos: ", joint_pos)
    # 创建发布者字典
    publishers = {}
    joint_names = ['joint012', 'joint612', 'joint712', 'joint812', 'joint412', 'joint456']
    for jid, joint_name in enumerate(joint_names):
        topic_name = '/swivel/joint{}_position_controller/command'.format(jid+1)
        publishers[joint_name] = rospy.Publisher(topic_name, Float64, queue_size=10)

    # 等待与ROS Master建立连接
    rospy.sleep(2)

    # 创建并发布0度（0弧度）位置的命令
    zero_position = Float64()
    zero_position.data = 0.0
    for pid, pub in enumerate(publishers.values()):
        pos = Float64(); pos.data = joint_pos[pid]
        pub.publish(pos)
        rospy.loginfo('Sent command to move {} to 0 radian position'.format(pub.name))

def move_hand_to_position(joint_pos):
    """
        No sleep here
    """
    print("desired joint pos: ", joint_pos)
    # 创建发布者字典
    publishers = {}
    joint_names = ['joint012', 'joint412', 'joint456', 'joint612', 'joint712', 'joint812']
    for jid, joint_name in enumerate(joint_names):
        topic_name = '/swivel/joint{}_position_controller/command'.format(jid+1)
        publishers[joint_name] = rospy.Publisher(topic_name, Float64, queue_size=10)

    # 创建并发布0度（0弧度）位置的命令
    zero_position = Float64()
    zero_position.data = 0.0
    for pid, pub in enumerate(publishers.values()):
        pos = Float64(); pos.data = joint_pos[pid]
        pub.publish(pos)
        # rospy.loginfo('Sent command to move {} to 0 radian position'.format(pub.name))

def get_joint_pos():
    """
        Return the current joint positions from /joint_states
    """
    joint_states = rospy.wait_for_message('/joint_states', JointState)
    return np.array(joint_states.position)

def read_hand_joint_states():
    topic_name = '/swivel/joint_states'
    joint_states = rospy.wait_for_message(topic_name, JointState)
    print("joint_states: ", joint_states)

def sweep_actuated_joints(n_steps=100, finger="0"):
    """
        Sweep the actuated joints and save the joint states
    """
    index_ = finger_to_index_map[finger]
    index_actuate_ = finger_to_actuated_index_map[finger]
    limits_ = finger_to_limits_map[finger]
    low_, high_ = limits_[0], limits_[1]

    j_pos_saved = np.zeros((n_steps, len(index_)))

    # sweep the actuated joint position
    for idx, ja_pos in enumerate(np.linspace(low_, high_, n_steps)[::-1]):
        print("index: {}".format(idx))
        # create jpos (actuated)
        ja_pos_all = np.zeros(6,)
        ja_pos_all[index_actuate_] = ja_pos

        # publish control command
        move_hand_to_position(ja_pos_all)

        # read joint pos
        j_pos = get_joint_pos()
        j_pos_saved[idx] = j_pos[index_]
        print("read jpos for finger {}: ".format(finger), j_pos[index_])

        rospy.sleep(0.5)

    return j_pos_saved



if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('test_gazebo_ros_control')

    # read_hand_joint_states()

    # move_hand_to_position(joint_pos=np.random.uniform(low=FINGER_LOWER_LIMITS, high=FINGER_UPPER_LIMITS))

    # rospy.sleep(3)

    # read_hand_joint_states()

    FINGER = "4-0"
    j_pos = sweep_actuated_joints(n_steps=100, finger=FINGER)
    np.save("./data/finger{}.npy".format(FINGER), j_pos)
