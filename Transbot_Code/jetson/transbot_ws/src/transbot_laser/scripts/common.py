#!/usr/bin/env python
# coding:utf-8
import rospy
from transbot_msgs.srv import *
from geometry_msgs.msg import Twist
from transbot_msgs.msg import JoyState


class ROSCtrl:
    def __init__(self):
        self.Joy_active = False
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        self.sub_JoyState = rospy.Subscriber('/JoyState', JoyState, self.JoyStateCallback)
        self.Buzzer_client = rospy.ServiceProxy("/Buzzer", Buzzer)

    def JoyStateCallback(self, msg):
        if not isinstance(msg, JoyState): return
        self.Joy_active = msg.state

    def Buzzer_srv(self, value):
        '''
        蜂鸣器控制
        :param value:
         [  0：   关闭,
            1：   一直响,
            >=10：响xx毫秒后自动关闭（value是10的倍数) ]
        Buzzer control
        : param value:
        [0: close,
        1: Keep ringing,
        & gt;=10: automatically closes after xx milliseconds (value is a multiple of 10)]
        '''
        self.Buzzer_client.wait_for_service()
        request = BuzzerRequest()
        request.buzzer = value
        try:
            response = self.Buzzer_client.call(request)
            if isinstance(response, BuzzerResponse): return response.result
        except Exception:
            rospy.loginfo("Buzzer error")
        return False

    def cancel(self):
        self.Buzzer_srv(0)
        self.pub_vel.publish(Twist())
        self.pub_vel.unregister()
        self.sub_JoyState.unregister()
        self.Buzzer_client.close()
        rospy.loginfo("Shutting down this node.")


class SinglePID:
    def __init__(self, P=0.1, I=0.0, D=0.1):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("init_pid: ", P, I, D)
        self.pid_reset()

    def Set_pid(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        print("set_pid: ", P, I, D)
        self.pid_reset()

    def pid_compute(self, target, current):
        self.error = target - current
        self.intergral += self.error
        self.derivative = self.error - self.prevError
        result = self.Kp * self.error + self.Ki * self.intergral + self.Kd * self.derivative
        self.prevError = self.error
        return result

    def pid_reset(self):
        self.error = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0
