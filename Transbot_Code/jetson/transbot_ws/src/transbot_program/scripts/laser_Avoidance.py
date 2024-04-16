#!/usr/bin/env python
# coding:utf-8
from Program_Ctrl import *
from geometry_msgs.msg import Twist


class laserAvoid:
    def __init__(self):
        self.angular = 1
        self.program_ctrl = ProgramCtrl()

    def process(self, front_warning, Left_warning, Right_warning, speed):
        twist = Twist()
        # 左正右负
        if front_warning > 10 and Left_warning > 10 and Right_warning > 10:
            # print ('1、右转')
            twist.linear.x = -0.15
            twist.angular.z = -self.angular
            self.program_ctrl.pub_cmdVel.publish(twist)
            sleep(0.1)
        elif front_warning > 10 and Left_warning <= 10 and Right_warning > 10:
            # print ('2、左转')
            twist.linear.x = 0
            twist.angular.z = self.angular
            self.program_ctrl.pub_cmdVel.publish(twist)
            sleep(0.1)
            if Left_warning > 10 and Right_warning <= 10:
                # print ('3、右转')
                twist.linear.x = 0
                twist.angular.z = -self.angular
                self.program_ctrl.pub_cmdVel.publish(twist)
                sleep(0.2)
        elif front_warning > 10 and Left_warning > 10 and Right_warning <= 10:
            # print ('4、右转')
            twist.linear.x = 0
            twist.angular.z = -self.angular
            self.program_ctrl.pub_cmdVel.publish(twist)
            sleep(0.1)
            if Right_warning <= 10 and Left_warning > 10:
                # print ('5、左转')
                twist.linear.x = 0
                twist.angular.z = self.angular
                self.program_ctrl.pub_cmdVel.publish(twist)
                sleep(0.2)
        elif front_warning > 10 and Left_warning < 10 and Right_warning < 10:
            # print ('6、左转')
            twist.linear.x = 0
            twist.angular.z = self.angular
            self.program_ctrl.pub_cmdVel.publish(twist)
            sleep(0.1)
        elif front_warning < 10 and Left_warning > 10 and Right_warning > 10:
            # print ('7、右转')
            twist.linear.x = 0
            twist.angular.z = -self.angular
            self.program_ctrl.pub_cmdVel.publish(twist)
            sleep(0.2)
        elif front_warning < 10 and Left_warning > 10 and Right_warning <= 10:
            # print ('8、右转')
            twist.linear.x = 0
            twist.angular.z = -self.angular
            self.program_ctrl.pub_cmdVel.publish(twist)
            sleep(0.1)
        elif front_warning < 10 and Left_warning <= 10 and Right_warning > 10:
            # print ('9、左转')
            twist.linear.x = 0
            twist.angular.z = self.angular
            self.program_ctrl.pub_cmdVel.publish(twist)
            sleep(0.1)
        elif front_warning <= 10 and Left_warning <= 10 and Right_warning <= 10:
            # print ('10、前进')
            twist.linear.x = speed
            twist.angular.z = 0
            self.program_ctrl.pub_cmdVel.publish(twist)
#         print (twist.linear.x,twist.angular.z)