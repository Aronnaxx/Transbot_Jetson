#!/usr/bin/env python
# coding:utf-8
from Program_Ctrl import *
from Single_PID import *
from dynamic_reconfigure.server import Server
from transbot_laser.cfg import laserWarningPIDConfig

class laserWarning:
    def __init__(self):
        self.Buzzer_state = False
        self.program_ctrl = ProgramCtrl()
        self.Warning_pid = SinglePID(3.0, 0.0, 3.0)
#         Server(laserWarningPIDConfig, self.dynamic_reconfigure_callback)

    def process(self, minDist, ResponseDist, minDistAngle):
        velocity = Twist()
        if -3 < minDistAngle < 3: minDistAngle = 0
        if minDist <= ResponseDist:
            if self.Buzzer_state == False:
                self.program_ctrl.Buzzer_srv(1)
                self.Buzzer_state = True
        else:
            if self.Buzzer_state == True:
                self.program_ctrl.Buzzer_srv(0)
                self.Buzzer_state = False
        velocity.linear.x = 0
        velocity.angular.z = self.Warning_pid.pid_compute(minDistAngle / 90.0, 0)
        self.program_ctrl.pub_cmdVel.publish(velocity)

    def Set_PID(self, pid):
        print ("laser_warning_pid: ", pid)
        self.Warning_pid.Set_pid(pid[0], pid[1], pid[2])

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.laserAngle = config['laserAngle']
        self.ResponseDist = config['ResponseDist']
        self.Warning_pid.Set_pid(config['ang_Kp'], config['ang_Ki'], config['ang_Kd'])
        return config