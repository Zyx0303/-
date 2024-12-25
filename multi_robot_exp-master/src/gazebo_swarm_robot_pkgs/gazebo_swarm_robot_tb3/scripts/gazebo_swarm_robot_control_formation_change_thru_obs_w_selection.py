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
    #obs = [6, 7, 8, 9]
    # 建立对象
    swarm_robot = SwarmRobot(index)
    #swarm_obs = SwarmRobot(obs)

    conv_ang = .02  # convergence angle
    conv_pos = .03  # convergence position
    MAX_W = 1  # 最大角速度 (rad/s)
    MIN_W = 0.02  # 最小角速度 (rad/s)
    MAX_V = 0.15  # 最大线速度 (m/s)
    MIN_V = 0.01  # 最小线速度 (m/s)
    k_w = 0.1  # w = k_w * ang_diff
    k_v = 0.1
    v_default = 0.1

    scale_0 = .5
    scale_1 = .5
    scale_2_min = .8

    x_ref_0 = np.array(
        [
            [-1.0,  0.0],
            [ 0.0, -1.0],
            [ 1.0,  0.0],
            [ 0.0,  1.0],
            [ 0.0,  0.0],
        ]
    )

    x_ref_1 = scale_1 * np.array(
        [
            [-1.0,  0.0],
            [ 0.0, -2.0],
            [ 1.0,  0.0],
            [ 0.0,  2.0],
            [ 0.0,  0.0],
        ]
    )

    x_ref_2 = np.array(
        [
            [-1.0, 0.0],
            [-0.5, 0.0],
            [ 0.0, 0.0],
            [ 0.5, 0.0],
            [ 1.0, 0.0],
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
    tar_theta = np.array([0.0] * swarm_robot.robot_num)

    ang_diff = np.array([1.0] * swarm_robot.robot_num)
    ang_diff_hist = np.array([1.0] * swarm_robot.robot_num)
    pos_diff = np.array([1.0] * swarm_robot.robot_num)

    # Get current pos/ang off all
    current_robot_pose = np.array(swarm_robot.get_robot_poses())
    print(current_robot_pose)
    #current_obs_pose = np.array(swarm_obs.get_robot_poses())
    #print(current_obs_pose)

    #avg_x_obs = np.mean(current_obs_pose[:, 0])
    #avg_y_obs = np.mean(current_obs_pose[:, 1])
    #width_obs = np.max(current_obs_pose[:, 1]) - np.min(current_obs_pose[:, 1])

    avg_x_obs = 1.0
    avg_y_obs = 0.0
    width_obs = 1.4

    
    # Calculate center point
    for i in range(swarm_robot.robot_num):
        cur_x[i] = current_robot_pose[i][0]
        cur_y[i] = current_robot_pose[i][1]
        cur_theta[i] = current_robot_pose[i][2]
        print('cur', i, cur_x[i], cur_y[i], cur_theta[i])

    avg_x = np.mean(cur_x)
    avg_y = np.mean(cur_y)
    print('avg', avg_x, avg_y)

    
    # Set target position for formation 1
    init_direc = np.array([0.0] * swarm_robot.robot_num)
    init_pos_diff = np.array([0.0] * swarm_robot.robot_num)

    for i in range(swarm_robot.robot_num):
        init_direc[i] = np.arctan2(cur_y[i] - avg_y_obs, cur_x[i] - avg_x)
        init_pos_diff[i] = np.sqrt((cur_y[i] - avg_y_obs) ** 2 + (cur_x[i] - avg_x) ** 2)

    #init_scale = np.max([scale_0, np.max(init_pos_diff)])
    init_scale = np.mean(init_pos_diff)
    print('init_scale', init_scale, 'width_obs/2', width_obs / 2)
    tar_0 = [avg_x, avg_y_obs] + x_ref_0 * init_scale
    init_direc[np.argmin(init_pos_diff)] = 3.2 # anything larger than pi is ok

    for i in range(swarm_robot.robot_num):
        index = np.argsort(init_direc)[i]
        tar_x[index] = tar_0[i][0]
        tar_y[index] = tar_0[i][1]
        
    # Calculate the direction to target 1
    for i in range(swarm_robot.robot_num):
        tar_theta[i] = np.arctan2(tar_y[i] - cur_y[i], tar_x[i] - cur_x[i])
        print('tar', i, tar_x[i], tar_y[i], tar_theta[i])
    
    ang_conv = False
    pos_conv = False

    # First turn to the direction to target
    while not ang_conv:
        for i in range(swarm_robot.robot_num):
            current_robot_pose = swarm_robot.get_robot_poses()

            cur_theta[i] = current_robot_pose[i][2]
            cur_x[i] = current_robot_pose[i][0]
            cur_y[i] = current_robot_pose[i][1]
            tar_theta[i] = np.arctan2(tar_y[i] - cur_y[i], tar_x[i] - cur_x[i])

            ang_diff[i] = tar_theta[i] - cur_theta[i]
            
            #print(i, 'ang_diff', ang_diff[i])
                    
            ang_conv = np.all(np.abs(ang_diff) <= conv_ang)
            #print('ang_conv', ang_conv)

            if ang_conv:
                swarm_robot.stop_robots()
                print('ang_conv')
                break 
            
            
            if abs(ang_diff[i]) > conv_ang:
                w = k_w * ang_diff[i]
                w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                if abs(ang_diff[i]) > 6.1:
                    ang_diff[i] = 0.0
                    swarm_robot.stop_robot(i)
                    swarm_robot.stop_robot(i)
                else:
                    swarm_robot.move_robot(i, 0, w)
                #print(i, 'w', w)
            else:
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
 
    rospy.sleep(0.05)

    # Then reach for target pos w/ ang fixing
    while not pos_conv:  
        for i in range(swarm_robot.robot_num):
            current_robot_pose = swarm_robot.get_robot_poses()
            
            cur_theta[i] = current_robot_pose[i][2]
            cur_x[i] = current_robot_pose[i][0]
            cur_y[i] = current_robot_pose[i][1]
            tar_theta[i] = np.arctan2(tar_y[i] - cur_y[i], tar_x[i] - cur_x[i])

            ang_diff_hist[i] = ang_diff[i]
            ang_diff[i] = tar_theta[i] - cur_theta[i]
            pos_diff[i] = np.sqrt(
                (tar_y[i] - cur_y[i]) ** 2 + (tar_x[i] - cur_x[i]) ** 2
            )

            #print(i, 'pos_diff', pos_diff[i])

            pos_conv = np.all(pos_diff <= conv_pos)
            #print('pos_conv', pos_conv)

            if pos_conv:
                swarm_robot.stop_robots()
                print('pos_conv')
                break

            if pos_diff[i] > conv_pos:
                v = k_v * pos_diff[i]
                v = swarm_robot.check_vel(v, MAX_V, MIN_V)
                if np.abs(ang_diff[i]) > np.pi / 2 and pos_diff[i] < 2 * conv_pos:
                    ang_diff[i] = 0.0
                    swarm_robot.stop_robot(i)
                    swarm_robot.stop_robot(i)
                else:
                    w = k_w * ang_diff[i]
                    w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                    swarm_robot.move_robot(i, v, w)  
                #print(i, 'v', v, 'w', w) 
            else:
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)

    rospy.sleep(0.05)

    swarm_robot.stop_robots()
    print('FORMATION 1 DONE')

    same_ang = 0.0
    ang_same = False
    while not ang_same:
        for i in range(swarm_robot.robot_num):
            current_robot_pose = swarm_robot.get_robot_poses()

            cur_theta[i] = current_robot_pose[i][2]
            
            ang_diff[i] = same_ang - cur_theta[i]
            
            #print(i, 'ang_diff', ang_diff[i])  
            
            ang_same = np.all(np.abs(ang_diff) <= conv_ang)
            #print('ang_same', ang_same)

            if ang_same:
                print('ang_same')
                break
            
            if abs(ang_diff[i]) > conv_ang:
                w = k_w * ang_diff[i]
                w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                if abs(ang_diff[i]) > 6.1:
                    ang_diff[i] = 0.0
                    swarm_robot.stop_robot(i)
                    swarm_robot.stop_robot(i)
                else:
                    swarm_robot.move_robot(i, 0, w)
                #print(i, 'w', w)
            else:
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
 
    rospy.sleep(0.05) 


    current_robot_pose = swarm_robot.get_robot_poses()
    x_max = np.max(current_robot_pose, axis=0)[0]
    for i in range(swarm_robot.robot_num):
        cur_x[i] = current_robot_pose[i][0]
        cur_y[i] = current_robot_pose[i][1]
        cur_theta[i] = current_robot_pose[i][2]
        print('cur', i, cur_x[i], cur_y[i], cur_theta[i])
    
    tar_x = tar_x + (avg_x_obs - x_max)
    print(tar_x)
    print(tar_y)

    detect_dist = .3

    while avg_x_obs > x_max + detect_dist:
        for i in range(swarm_robot.robot_num):
            current_robot_pose = swarm_robot.get_robot_poses()

            cur_theta[i] = current_robot_pose[i][2]
            cur_x[i] = current_robot_pose[i][0]
            cur_y[i] = current_robot_pose[i][1]
            tar_theta[i] = np.arctan2(tar_y[i] - cur_y[i], tar_x[i] - cur_x[i])
            ang_diff[i] = tar_theta[i] - cur_theta[i]
            x_max = np.max(current_robot_pose, axis=0)[0]

            w = k_w * ang_diff[i]
            w = swarm_robot.check_vel(w, MAX_W, MIN_W)
            print(i, 'ang_diff', ang_diff[i], 'w', w)
            swarm_robot.move_robot(i, v_default, w)
            
    for i in range(swarm_robot.robot_num):
        swarm_robot.stop_robots()

    current_robot_pose = swarm_robot.get_robot_poses()
    for i in range(swarm_robot.robot_num):
        cur_x[i] = current_robot_pose[i][0]
        cur_y[i] = current_robot_pose[i][1]
        cur_theta[i] = current_robot_pose[i][2]
        print('cur', i, cur_x[i], cur_y[i], cur_theta[i])

    avg_x = np.mean(cur_x)
    avg_y = np.mean(cur_y)
    print('avg', avg_x, avg_y)

    if init_scale >= width_obs / 2 - .1:
        # Set target position for formation 1
        tar_1 = [avg_x, avg_y_obs] + x_ref_1

        tar_x[np.argmin(init_pos_diff)] = tar_1[4][0]
        tar_y[np.argmin(init_pos_diff)] = tar_1[4][1]

        tar_x[np.argmin(cur_x)] = tar_1[0][0]
        tar_y[np.argmin(cur_x)] = tar_1[0][1]

        tar_x[np.argmin(cur_y)] = tar_1[1][0]
        tar_y[np.argmin(cur_y)] = tar_1[1][1]

        tar_x[np.argmax(cur_x)] = tar_1[2][0]
        tar_y[np.argmax(cur_x)] = tar_1[2][1]

        tar_x[np.argmax(cur_y)] = tar_1[3][0]
        tar_y[np.argmax(cur_y)] = tar_1[3][1]
    else:
        # Set target position for formation 2
        tar_2 = [avg_x, avg_y] + x_ref_2 * np.max([scale_2_min, init_scale])

        for i in range(swarm_robot.robot_num):
            index = np.argsort(cur_x)[i]
            tar_x[index] = tar_2[i][0]
            tar_y[index] = tar_2[i][1]

    print(tar_1)
    
    # Calculate the direction to target 2
    for i in range(swarm_robot.robot_num):
        tar_theta[i] = np.arctan2(tar_y[i] - cur_y[i], tar_x[i] - cur_x[i])
        print('tar', i, tar_x[i], tar_y[i], tar_theta[i])

    ang_conv = False
    pos_conv = False

    # First turn to the direction to target 2
    while not ang_conv:
        for i in range(swarm_robot.robot_num):
            current_robot_pose = swarm_robot.get_robot_poses()

            cur_theta[i] = current_robot_pose[i][2]
            cur_x[i] = current_robot_pose[i][0]
            cur_y[i] = current_robot_pose[i][1]
            tar_theta[i] = np.arctan2(tar_y[i] - cur_y[i], tar_x[i] - cur_x[i])

            ang_diff[i] = tar_theta[i] - cur_theta[i]
            
            #print(i, 'ang_diff', ang_diff[i])
                    
            ang_conv = np.all(np.abs(ang_diff) <= conv_ang)
            #print('ang_conv', ang_conv)

            if ang_conv:
                swarm_robot.stop_robots()
                print('ang_conv')
                break            
            
            if abs(ang_diff[i]) > conv_ang:
                w = k_w * ang_diff[i]
                w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                if abs(ang_diff[i]) > 6.1:
                    ang_diff[i] = 0.0
                    swarm_robot.stop_robot(i)
                    swarm_robot.stop_robot(i)
                else:
                    swarm_robot.move_robot(i, 0, w)
                #print(i, 'w', w)
            else:
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
 
    rospy.sleep(0.05)

    # Then reach for target pos w/ ang fixing
    while not pos_conv:  
        for i in range(swarm_robot.robot_num):
            current_robot_pose = swarm_robot.get_robot_poses()

            cur_theta[i] = current_robot_pose[i][2]
            cur_x[i] = current_robot_pose[i][0]
            cur_y[i] = current_robot_pose[i][1]
            tar_theta[i] = np.arctan2(tar_y[i] - cur_y[i], tar_x[i] - cur_x[i])

            ang_diff[i] = tar_theta[i] - cur_theta[i]
            pos_diff[i] = np.sqrt(
                (tar_y[i] - cur_y[i]) ** 2 + (tar_x[i] - cur_x[i]) ** 2
            )

            #print(i, 'pos_diff', pos_diff[i])

            pos_conv = np.all(pos_diff <= conv_pos)
            #print('pos_conv', pos_conv)

            if pos_conv:
                swarm_robot.stop_robots()
                print('pos_conv')
                break

            if pos_diff[i] > conv_pos:
                v = k_v * pos_diff[i]
                v = swarm_robot.check_vel(v, MAX_V, MIN_V)
                if np.abs(ang_diff[i]) > np.pi / 2 and pos_diff[i] < 2 * conv_pos:
                    ang_diff[i] = 0.0
                    swarm_robot.stop_robot(i)
                    swarm_robot.stop_robot(i)
                    swarm_robot.stop_robot(i)
                    swarm_robot.stop_robot(i)
                    swarm_robot.stop_robot(i)
                else:
                    w = k_w * ang_diff[i]
                    w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                    swarm_robot.move_robot(i, v, w)  
                #print(i, 'v', v, 'w', w) 
            else:
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)


    rospy.sleep(0.05)
    
    print('FORMATION 2 DONE')
    
    same_ang = 0.0
    ang_same = False
    while not ang_same:
        for i in range(swarm_robot.robot_num):
            current_robot_pose = swarm_robot.get_robot_poses()

            cur_theta[i] = current_robot_pose[i][2]
            
            ang_diff[i] = same_ang - cur_theta[i]
            
            #print(i, 'ang_diff', ang_diff[i])  
            
            ang_same = np.all(np.abs(ang_diff) <= conv_ang)
            #print('ang_same', ang_same)

            if ang_same:
                swarm_robot.stop_robot(i)
                print('ang_same')
                break
            
            if abs(ang_diff[i]) > conv_ang:
                w = k_w * ang_diff[i]
                w = swarm_robot.check_vel(w, MAX_W, MIN_W)
                if abs(ang_diff[i]) > 6.1:
                    ang_diff[i] = 0.0
                    swarm_robot.stop_robot(i)
                    swarm_robot.stop_robot(i)
                else:
                    swarm_robot.move_robot(i, 0, w)
                #print(i, 'w', w)
            else:
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
                swarm_robot.stop_robot(i)
 
    rospy.sleep(0.05)        

    current_robot_pose = swarm_robot.get_robot_poses()
    x_min = np.min(current_robot_pose, axis=0)[0]
    for i in range(swarm_robot.robot_num):
        cur_x[i] = current_robot_pose[i][0]
        cur_y[i] = current_robot_pose[i][1]
        cur_theta[i] = current_robot_pose[i][2]
        print('cur', i, cur_x[i], cur_y[i], cur_theta[i])
    
    tar_x = tar_x + (avg_x_obs - x_min) + 2 * detect_dist

    while avg_x_obs > x_min - detect_dist:
        for i in range(swarm_robot.robot_num):
            current_robot_pose = swarm_robot.get_robot_poses()

            cur_theta[i] = current_robot_pose[i][2]
            cur_x[i] = current_robot_pose[i][0]
            cur_y[i] = current_robot_pose[i][1]
            tar_theta[i] = np.arctan2(tar_y[i] - cur_y[i], tar_x[i] - cur_x[i])
            ang_diff[i] = tar_theta[i] - cur_theta[i]
            x_min = np.min(current_robot_pose, axis=0)[0]

            w = k_w * ang_diff[i]
            w = swarm_robot.check_vel(w, MAX_W, MIN_W)
            swarm_robot.move_robot(i, v_default, w)
            
    for i in range(swarm_robot.robot_num):
        swarm_robot.stop_robots()
    
        
        
        
        
        

    # 停止所有机器人
    swarm_robot.stop_robots()
    swarm_robot.stop_robots()
    swarm_robot.stop_robots()
    swarm_robot.stop_robots()
    swarm_robot.stop_robots()
    swarm_robot.stop_robots()

    rospy.loginfo("Succeed!")


if __name__ == "__main__":
    main()
