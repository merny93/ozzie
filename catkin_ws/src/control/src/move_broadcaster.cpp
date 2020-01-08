#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int32MultiArray.h"


void moveCallback(const geometry_msgs::Twist& msg){
    static ros::NodeHandle n;
    static ros::Publisher move_pub = n.advertise<std_msgs::Int32MultiArray>("move", 5); // buffer of 5

    double linVel = msg.linear.x;
    double angVel = msg.angular.z;

    double vLeft = linVel - angVel; // left wheel
    double vRight = linVel + angVel; // right wheel

    if (abs(vLeft) <= 0.5 && abs(vRight) <= 0.5) {
        std_msgs::Int32MultiArray msg;
        msg.data.clear();
        
        msg.data.push_back(int(1000*vRight)); // push Right velocity first
        msg.data.push_back(int(1000*vLeft));

        move_pub.publish(msg);
    } else {
        ROS_WARN("Too high speed for robot");
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "move_broadcaster");

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/cmd_vel", 10, &moveCallback);

    ros::spin();
    return 0;
};