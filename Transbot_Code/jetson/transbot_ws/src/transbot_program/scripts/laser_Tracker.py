#!/usr/bin/env python
# coding:utf-8
from Single_PID import *
from Program_Ctrl import *
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from transbot_laser.cfg import laserTrackerPIDConfig

class laserTracker:
    def __init__(self):
        self.Buzzer_state = False
        self.program_ctrl = ProgramCtrl()
        self.lin_pid = SinglePID(3.0, 0.0, 0.5)
        self.ang_pid = SinglePID(4.0, 0.0, 1.0)
        # Server(laserTrackerPIDConfig, self.dynamic_reconfigure_callback)

    def process(self, minDist, ResponseDist, laserTrackerAngle, back_state):
        velocity = Twist()
        if -3 < laserTrackerAngle < 3: laserTrackerAngle = 0
        if abs(minDist - ResponseDist) < 0.1: minDist = ResponseDist
        velocity.linear.x = -self.lin_pid.pid_compute(ResponseDist, minDist)
        if minDist <= 0.4:
            velocity.linear.x = 0
            if self.Buzzer_state == False:
                self.program_ctrl.Buzzer_srv(1)
                self.Buzzer_state = True
        if back_state == False:
            if velocity.linear.x < 0: velocity.linear.x = 0
            if self.Buzzer_state == False:
                self.program_ctrl.Buzzer_srv(1)
                self.Buzzer_state = True
        if self.Buzzer_state == True:
            self.program_ctrl.Buzzer_srv(0)
            self.Buzzer_state = False
        velocity.angular.z = self.ang_pid.pid_compute(laserTrackerAngle / 90.0, 0)
        self.program_ctrl.pub_cmdVel.publish(velocity)

    def Set_PID(self, lin_pid, ang_pid):
        self.lin_pid.Set_pid(lin_pid[0], lin_pid[1], lin_pid[2])
        self.ang_pid.Set_pid(ang_pid[0], ang_pid[1], ang_pid[2])

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.laserAngle = config['laserAngle']
        self.priorityAngle = config['priorityAngle']
        self.ResponseDist = config['ResponseDist']
        self.lin_pid.Set_pid(config['lin_Kp'], config['lin_Ki'], config['lin_Kd'])
        self.ang_pid.Set_pid(config['ang_Kp'], config['ang_Ki'], config['ang_Kd'])
        return config