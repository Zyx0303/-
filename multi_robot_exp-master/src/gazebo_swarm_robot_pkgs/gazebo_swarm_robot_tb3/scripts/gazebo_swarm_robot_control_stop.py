#! /usr/bin/env python3
# encoding: utf-8
import numpy as np
import rospy
from gazebo_swarm_robot_control import SwarmRobot


def main():
    # 初始化节点
    rospy.init_node("swarm_robot_control_angle")
    # 机器人的id
    index = [1, 2, 3, 4, 5, 6, 7, 8]
    # 建立对象
    swarm_robot = SwarmRobot(index)
    # 停止所有机器人
    
    for i in range(1000):
        swarm_robot.stop_robots()


if __name__ == "__main__":
    main()
