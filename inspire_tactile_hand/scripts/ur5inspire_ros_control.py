#!/usr/bin/env python
"""
    Run mylaunch.launch first, and control the robot
"""

import rospy

import numpy as np

import actionlib
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


FINGER_UPPER_LIMITS = [0.0089, 0.00875, 0.0083, 0.0079, 0.006, 0.0042]
FINGER_LOWER_LIMITS = [-0.001, -0.001, -0.0015, -0.002, -0.0027, -0.0058]

def move_ur5_start_position():
    # 定义轨迹点
    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # UR5起始位置的关节角度（弧度）
    point.time_from_start = rospy.Duration(5.0)  # 从开始到达该点的时间

    # 定义轨迹
    trajectory = JointTrajectory()
    trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    trajectory.points.append(point)

    # 发布轨迹
    pub = rospy.Publisher('/ur5_inspire/eff_joint_traj_controller/command', JointTrajectory, queue_size=1)
    
    # 稍等一会儿，直到连接建立
    rospy.sleep(1)

    # 发布轨迹命令
    pub.publish(trajectory)
    rospy.loginfo('Sent UR5 to start position.')

def move_hand_to_zero_position(joint_pos):
    print("desired joint pos: ", joint_pos)
    # 创建发布者字典
    publishers = {}
    joint_names = ['joint012', 'joint412', 'joint456', 'joint612', 'joint712', 'joint812']
    for jid, joint_name in enumerate(joint_names):
        topic_name = '/ur5_inspire/joint{}_position_controller/command'.format(jid+1)
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


if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('test_gazebo_ros_control')

    try:
        move_ur5_start_position()
    except rospy.ROSInterruptException:
        pass

    try:
        move_hand_to_zero_position(joint_pos=np.zeros(6,))
        # move_hand_to_zero_position(joint_pos=np.random.uniform(low=FINGER_LOWER_LIMITS, high=FINGER_UPPER_LIMITS))
    except rospy.ROSInterruptException:
        pass