#! /usr/bin/env python3
# encoding: utf-8
import numpy as np
import rospy
from gazebo_swarm_robot_control import SwarmRobot


def main():
    # 初始化节点
    rospy.init_node("swarm_robot_control_angle")
    # 机器人的id
    index = [1, 2, 3, 4, 5]
    # 建立对象
    swarm_robot = SwarmRobot(index)

    conv_ang = .1  # convergence angle
    conv_pos = .05  # convergence position
    MAX_W = 1  # 最大角速度 (rad/s)
    MIN_W = 0.02  # 最小角速度 (rad/s)
    MAX_V = 0.2  # 最大线速度 (m/s)
    MIN_V = 0.01  # 最小线速度 (m/s)
    k_w = 0.1  # w = k_w * ang_diff
    k_v = 0.1

    # Laplace matrix
    lap = np.array(
        [
            [4, -1, -1, -1, -1],
            [-1, 4, -1, -1, -1],
            [-1, -1, 4, -1, -1],
            [-1, -1, -1, 4, -1],
            [-1, -1, -1, -1, 4],
        ]
    )

    scale_1 = .75
    scale_2 = 1.0

    x_ref_1 = scale_1 * np.array(
        [
            [-0.58778525, -0.80901699],
            [0.58778525, -0.80901699],
            [0.95105652, 0.30901699],
            [0.0,1.0],
            [-0.95105652, 0.30901699],
        ]
    )

    x_ref_2 = scale_2 * np.array(
        [
            [-1.0, 0.0],
            [- .5,  .5],
            [ 0.0, 1.0],
            [  .5,  .5],
            [ 1.0, 0.0]
        ]
    )

    # Save current pos/ang, target and difference 
    cur_x = np.array([0.0] * swarm_robot.robot_num)
    cur_y = np.array([0.0] * swarm_robot.robot_num)
    cur_theta = np.array([0.0] * swarm_robot.robot_num)

    tar_x1 = np.array([0.0] * swarm_robot.robot_num)
    tar_y1 = np.array([0.0] * swarm_robot.robot_num)
    tar_x2 = np.array([0.0] * swarm_robot.robot_num)
    tar_y2 = np.array([0.0] * swarm_robot.robot_num)
    tar_x = np.array([0.0] * swarm_robot.robot_num)
    tar_y = np.array([0.0] * swarm_robot.robot_num)
    tar_trans = np.array([0.0] * swarm_robot.robot_num)
    tar_x_trans = np.array([0.0] * swarm_robot.robot_num)
    tar_y_trans = np.array([0.0] * swarm_robot.robot_num)
    tar_theta = np.array([0.0] * swarm_robot.robot_num)

    ang_diff = np.array([1.0] * swarm_robot.robot_num)
    pos_diff = np.array([1.0] * swarm_robot.robot_num)

    # Get current pos/ang off all
    current_robot_pose = swarm_robot.get_robot_poses()
    print(current_robot_pose)

    current_robot_pose = swarm_robot.get_robot_poses()
    print(current_robot_pose) 


    # Calculate center point
    avg_x = 0.0
    avg_y = 0.0
    for i in range(swarm_robot.robot_num):
        cur_x[i] = current_robot_pose[i][0]
        cur_y[i] = current_robot_pose[i][1]
        cur_theta[i] = current_robot_pose[i][2]
        avg_x = avg_x + cur_x[i]
        avg_y = avg_y + cur_y[i]
        print('cur', i, cur_x[i], cur_y[i], cur_theta[i])

    avg_x = avg_x / swarm_robot.robot_num
    avg_y = avg_y / swarm_robot.robot_num
    print('avg', avg_x, avg_y)

    # Set target position for formation 1
    tar_1 = [avg_x, avg_y] + x_ref_1

    init_direc = np.array([0.0] * swarm_robot.robot_num)

    for i in range(swarm_robot.robot_num):
        init_direc[i] = np.arctan2(cur_y[i] - avg_y, cur_x[i] - avg_x)

    for i in range(swarm_robot.robot_num):
        index = np.argsort(init_direc)[i]
        tar_x1[index] = tar_1[i][0]
        tar_y1[index] = tar_1[i][1]
        

    # Set target position for formation 2
    tar_2 = [avg_x, avg_y] + x_ref_2

    for i in range(swarm_robot.robot_num):
        index = np.argsort(cur_x)[i]
        tar_x2[index] = tar_2[i][0]
        tar_y2[index] = tar_2[i][1]

    # Calculate Error of each formation and make selection
    error_1 = 0.0
    error_2 = 0.0
    selection = 0

    for i in range(swarm_robot.robot_num):
        for j in range(swarm_robot.robot_num):
            error_1 = error_1 + ((cur_x[i] - cur_x[j]) ** 2 + (cur_y[i] - cur_y[j]) ** 2 - (tar_x1[i] - tar_x1[j]) ** 2 - (tar_y1[i] - tar_y1[j]) ** 2) ** 2
            error_2 = error_2 + ((cur_x[i] - cur_x[j]) ** 2 + (cur_y[i] - cur_y[j]) ** 2 - (tar_x2[i] - tar_x2[j]) ** 2 - (tar_y2[i] - tar_y2[j]) ** 2) ** 2

    if error_1 < error_2:
        selection = 1
        tar_x = tar_x1
        tar_y = tar_y1
        print('Formation 1')
    else:
        selection = 2
        tar_x = tar_x2
        tar_y = tar_y2
        print('Formation 2')
        

    # 停止所有机器人
    swarm_robot.stop_robots()

    rospy.loginfo("Succeed!")


if __name__ == "__main__":
    main()
