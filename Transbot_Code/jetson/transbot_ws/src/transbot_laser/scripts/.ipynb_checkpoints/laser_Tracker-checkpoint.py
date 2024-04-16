#!/usr/bin/env python
# coding:utf-8
import numpy as np
from common import *
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from transbot_laser.cfg import laserTrackerPIDConfig


class laserTracker:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.laserAngle = 65
        self.priorityAngle = 35  # 40
        self.Moving = False
        self.switch = False
        self.ros_ctrl = ROSCtrl()
        self.Buzzer_state = False
        self.lin_pid = SinglePID(3.0, 0.0, 0.5)
        self.ang_pid = SinglePID(4.0, 0.0, 1.0)
        self.ResponseDist = rospy.get_param('~targetDist', 1.0)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan)
        Server(laserTrackerPIDConfig, self.dynamic_reconfigure_callback)

    def cancel(self):
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        rospy.loginfo("Shutting down this node.")

    def registerScan(self, scan_data):
        # 记录激光扫描并发布最近物体的位置（或指向某点）
        # Record the laser scan and publish the position of the nearest object (or point to a point)
        ranges = np.array(scan_data.ranges)
        back_state = True
        offset = 0.5
        frontMinDistList = []
        frontIDList = []
        minDistList = []
        minDistIDList = []
        for i in np.argsort(ranges):
            if len(np.array(scan_data.ranges)) == 720:
                # 通过清除不需要的扇区的数据来保留有效的数据
                # Keep valid data by purging data from unneeded sectors
                if 270 < i < 450:
                    if ranges[i] < 0.5 : back_state = False
                elif 0 < i < self.priorityAngle * 2:
                    if ranges[i] < (self.ResponseDist + offset):
                        frontMinDistList.append(ranges[i])
                        frontIDList.append(i / 2)
                elif (720 - self.priorityAngle * 2) <= i :
                    if ranges[i] <= (self.ResponseDist + offset):
                        frontMinDistList.append(ranges[i])
                        frontIDList.append(i / 2 - 360)
                elif i < self.laserAngle * 2:
                    minDistList.append(ranges[i])
                    minDistIDList.append(i / 2)
                elif (720 - self.laserAngle * 2) <= i :
                    minDistList.append(ranges[i])
                    minDistIDList.append(i / 2 - 360)
            if len(np.array(scan_data.ranges)) == 360:
                # 通过清除不需要的扇区的数据来保留有效的数据
                # Keep valid data by purging data from unneeded sectors
                if 135 < i < 225:
                    if ranges[i] < 0.5 : back_state = False
                elif i < self.priorityAngle:
                    if ranges[i] < (self.ResponseDist + offset):
                        frontMinDistList.append(ranges[i])
                        frontIDList.append(i)
                elif (360 - self.priorityAngle) <= i :
                    if ranges[i] <= (self.ResponseDist + offset):
                        frontMinDistList.append(ranges[i])
                        frontIDList.append(i - 360)
                elif i < self.laserAngle:
                    minDistList.append(ranges[i])
                    minDistIDList.append(i)
                elif (360 - self.laserAngle) <= i :
                    minDistList.append(ranges[i])
                    minDistIDList.append(i - 360)
        # 找到最小距离和最小距离对应的ID
        # Find the minimum distance and the ID corresponding to the minimum distance
        if len(frontIDList) != 0:
            minDist = min(frontMinDistList)
            minDistID = frontIDList[frontMinDistList.index(minDist)]
        else:
            minDist = min(minDistList)
            minDistID = minDistIDList[minDistList.index(minDist)]
        if not (scan_data.range_min < minDist < scan_data.range_max): rospy.logwarn('laser no object found')
        else: self.process(back_state, minDist, minDistID)

    def process(self, back_state, minDist, minDistID):
        if self.ros_ctrl.Joy_active or self.switch == True:
            if self.Moving == True:
                self.ros_ctrl.pub_vel.publish(Twist())
                self.ros_ctrl.Buzzer_srv(0)
                self.Moving = not self.Moving
            return
        self.Moving = True
        velocity = Twist()
        if -3 < minDistID < 3: minDistID = 0
        if abs(minDist - self.ResponseDist) < 0.1: minDist = self.ResponseDist
        velocity.linear.x = -self.lin_pid.pid_compute(self.ResponseDist, minDist)
        if back_state == False:
            # 后方有障碍物，鸣笛，停止后退。
            # Obstruction behind, honk, stop retreating.
            if velocity.linear.x < 0: velocity.linear.x = 0
            if self.Buzzer_state == False:
                self.ros_ctrl.Buzzer_srv(1)
                self.Buzzer_state = True
        if self.Buzzer_state == True:
            self.ros_ctrl.Buzzer_srv(0)
            self.Buzzer_state = False
        velocity.angular.z = self.ang_pid.pid_compute(minDistID / 90.0, 0)
        self.ros_ctrl.pub_vel.publish(velocity)
        # rospy.loginfo('minDist: {}, velocity.linear.x: {}'.format(minDist, velocity.linear.x))
        # rospy.loginfo('minDistID: {}, angularSpeed: {}'.format(minDistID, velocity.angular.z))

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.laserAngle = config['laserAngle']
        self.priorityAngle = config['priorityAngle']
        self.ResponseDist = config['ResponseDist']
        self.lin_pid.Set_pid(config['lin_Kp'], config['lin_Ki'], config['lin_Kd'])
        self.ang_pid.Set_pid(config['ang_Kp'], config['ang_Ki'], config['ang_Kd'])
        return config

if __name__ == '__main__':
    rospy.init_node('laser_Tracker', anonymous=False)
    tracker = laserTracker()
    rospy.spin()
