#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

int main(int argc, char **argv) {
    //ROS节点初始化 Initialize  ROS node
    ros::init(argc, argv, "random_move_cpp");
    //创建节点句柄  Create node handle
    ros::NodeHandle n;
	// 设置线程 Set thread
	ros::AsyncSpinner spinner(1);
	// 开启线程 Start thread
	spinner.start();
    //初始化机械臂 Initialize the robotic arm
    moveit::planning_interface::MoveGroupInterface dofbot("arm");
	// 设置最大速度  Set maximum speed
    dofbot.setMaxVelocityScalingFactor(1.0);
    // 设置最大加速度 Set maximum acceleration
    dofbot.setMaxAccelerationScalingFactor(1.0);
    //设置目标点 Set the target point
//    dofbot.setNamedTarget("pose1");
//    //开始移动 Start 
//    dofbot.move();
//    sleep(0.1);
    while (not ros::isShuttingDown()){
    	//设置随机目标点 Set random target points
    	dofbot.setRandomTarget();
    	//开始移动 Start 
    	dofbot.move();
    	sleep(0.5);
    }
    return 0;
}
