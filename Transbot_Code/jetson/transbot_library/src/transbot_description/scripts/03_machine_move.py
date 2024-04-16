#!/usr/bin/env python
# coding: utf-8
'''
在从机中执行此段代码-->订阅发布话题为"joint_states"的各关节角度,驱动真机移动
Execute this code in the slave device-->Subscribe and publish the angle of each joint whose topic is "joint_states" to drive the real machine to move
'''
import rospy
from math import pi
from Transbot_Lib import Transbot
from sensor_msgs.msg import JointState
import sys
sys.path.append("/home/jetson/Transbot/transbot")
from arm_transbot import Transbot_ARM
# 弧度转角度 Radian to angle
RA2DE = 180 / pi

'''
msg.name:  ['camera_joint1', 'camera_joint2', 'arm_joint1', 'arm_joint2', 'Jaws_joint', 
             'right_joint2', 'right_joint3', 'left_joint1', 'left_joint2', 'left_joint3']
msg.position:  (0.0, 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0)
'''


def JointTopic(msg):
    # 如果不是该话题的数据直接返回 If it is not the data of the topic, return directly
    if not isinstance(msg, JointState): return
    if len(msg.position) == 10:
        joints1 = (msg.position[0] * RA2DE) + 90
        joints2 = -(msg.position[1] * RA2DE) + 90
        joints3 = (msg.position[2] * RA2DE) + 225
        joints4 = (msg.position[3] * RA2DE) + 45
        joints5 = (msg.position[4] * RA2DE) * 5 / 3 + 180
        bot.set_pwm_servo(1, int(joints1))
        if int(joints2) > 140: joints2 = 140
        bot.set_pwm_servo(2, int(joints2))
        bot.set_uart_servo_angle_array(joints3, joints4, joints5)
    elif len(msg.position) == 9:
        joints1 = (msg.position[0] * RA2DE) + 90
        joints2 = (msg.position[1] * RA2DE) + 225
        joints3 = (msg.position[2] * RA2DE) + 45
        joints4 = (msg.position[3] * RA2DE) * 5 / 3 + 180
        bot.set_pwm_servo(1, int(joints1))
        bot.set_uart_servo_angle_array(joints2, joints3, joints4)


if __name__ == '__main__':
    bot_arm = Transbot_ARM()
    bot_arm_offset = bot_arm.get_arm_offset()
    bot = Transbot(arm_offset=bot_arm_offset)
    rospy.init_node("ros_transbot")
    bot.create_receive_threading()
    subscriber = rospy.Subscriber("/joint_states", JointState, JointTopic)
    # bot.set_uart_servo_angle(9, 90, 500)
    # while not rospy.is_shutdown():
    #     rospy.sleep(0.1)
    #     joints = bot.get_uart_servo_angle_array()
    #     print joints
    rospy.spin()