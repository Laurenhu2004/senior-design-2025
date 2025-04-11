#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from pymycobot.mycobot import MyCobot
import time
from pymycobot.genre import Angle

mc = MyCobot("/dev/ttyACM0", 115200)      


def open_gripper():  
    # mc.send_angles([0, 0, 0, 0, 0, 0], 50)  
    # time.sleep(5)
    # mc.send_angles([-76, -6, -68, 74, -16, -2], 50)   #去目标点进行夹取
    # time.sleep(3)
    mc.set_gripper_state(0,80)   #夹爪控制.夹爪合拢状态
    time.sleep(3)
    #mc.send_angles([83.05, -103.53, 143.87, -72.15, 3.25, 46.49], 50)  #收缩
    #time.sleep(5)

def close_gripper():
    # time.sleep(1)
    # mc.send_angles([92.02, -39.99, -31.46, 52.73, -2.02, 46.14], 50)  #去到目标点投放
    # time.sleep(3)
    mc.set_gripper_state(1,80)   #夹爪控制.夹爪打开

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('pick_place')
    
    open_gripper()  
    rospy.loginfo("target 1 done")

    # client.wait_for_result()

    close_gripper()
    rospy.loginfo("target 2 done")

    
    # rospy.spin()


