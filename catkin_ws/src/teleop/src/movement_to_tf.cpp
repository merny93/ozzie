// Converts movement commands into tf transformations (for SLAM)

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

const float DISTANCE_BETWEEN_WHEELS = 0.2; // m

double x = 0.0;
double y = 0.0;
double th = 0.0;

ros::Time current_time, last_time;

void poseCallback(const geometry_msgs::Twist& msg){
    static ros::NodeHandle n;
    static tf::TransformBroadcaster odom_broadcaster;
    static ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();

    double linVel = msg.linear.x;
    double angVel = msg.angular.z;


    double vLeft = linVel - angVel; // left wheel
    double vRight = linVel + angVel; // right wheel
    double v = (vLeft + vRight)/2;
    double vx = v*cos(th);
    double vy = v*sin(th);
    double vth = (vRight - vLeft) / DISTANCE_BETWEEN_WHEELS;
    
    double dx = vx*dt;
    double dy = vy*dt;
    double dth = vth*dt;

    x += dx;
    y += dy;
    th += dth;
    // ROS_INFO("x: [%f]", x);


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "cmd_vel_tf");

    ros::NodeHandle node;
    
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Subscriber sub = node.subscribe("/cmd_vel", 10, &poseCallback);

    ros::spin();
    return 0;
};