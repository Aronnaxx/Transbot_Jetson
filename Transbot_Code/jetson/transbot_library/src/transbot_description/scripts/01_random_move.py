#!/usr/bin/env python
# coding: utf-8
from time import sleep
import rospy
from moveit_commander.move_group import MoveGroupCommander

if __name__ == '__main__':
    # 初始化节点 Initialize node
    rospy.init_node("transbot_set_move")
    # 初始化机械臂 Initialize the robotic arm
    dofbot = MoveGroupCommander("arm")
    # 当运动规划失败后，允许重新规划 When motion planning fails, re-planning is allowed
    dofbot.allow_replanning(True)
    dofbot.set_planning_time(5)
    # 尝试规划的次数 Number of planning attempts
    dofbot.set_num_planning_attempts(10)
    # 设置允许目标位置误差 Set the allowable target position error
    dofbot.set_goal_position_tolerance(0.01)
    # 设置允许目标姿态误差 Set the allowable target attitude error
    dofbot.set_goal_orientation_tolerance(0.01)
    # 设置允许目标误差 Set the allowable target error
    dofbot.set_goal_tolerance(0.01)
    # 设置最大速度 Set maximum speed
    dofbot.set_max_velocity_scaling_factor(1.0)
    # 设置最大加速度 Set maximum acceleration
    dofbot.set_max_acceleration_scaling_factor(1.0)
    while not rospy.is_shutdown():
        # 设置随机目标点 Set random target points
        dofbot.set_random_target()
        # 开始运动 Start 
        dofbot.go()
        sleep(0.5)
        # 设置"up"为目标点  Set "up" as the target point
        #dofbot.set_named_target("pose1")
        #dofbot.go()
        #sleep(0.5)
        # 设置"down"为目标点  Set "down" as the target point
        #dofbot.set_named_target("pose2")
        #dofbot.go()
        #sleep(0.5)
