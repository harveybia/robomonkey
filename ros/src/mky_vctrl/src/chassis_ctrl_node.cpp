/**
 * @file chassis_ctrl.cpp
 * @author Haowen Shi
 * @date 7 Mar 2019
 * @brief ROS wrapper for RoboMonkey vehicle control.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pthread.h>
#include "mkycom.h"

#define MSG_QUEUE_SIZE (1)

int16_t lwheel_vtarget = 0;
int16_t rwheel_vtarget = 0;

void print_usage(void) {
  printf("usage: rosrun mky_vctrl chassis_ctrl <tty_path>\n");
}

void vtarget_callback(geometry_msgs::Twist vtarget) {
    lwheel_vtarget = vtarget.linear.x + vtarget.angular.z;
    rwheel_vtarget = vtarget.linear.x - vtarget.angular.z;

    mkycom_send_chassis_command(lwheel_vtarget, rwheel_vtarget);
}

int main(int argc, char **argv) {
    int err;
    pthread_t comm_thread;

    if (argc < 2) {
        print_usage();
        ROS_FATAL("[param]: check param usage");
        return -1;
    }

    if (mkycom_init(argv[1]) < 0) {
        ROS_FATAL("[failed]: mkycom_init() failed, check tty_path");
        return -2;
    }

    // create and start feedback decoding thread
    pthread_create(&comm_thread, NULL, mkycom_recv_thread, NULL);

    ros::init(argc, argv, "chassis_ctrl");

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe(
        "mky_vtarget", MSG_QUEUE_SIZE, vtarget_callback);

    ros::spin();

    return 0;
}