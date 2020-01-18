// Good link to understand the tf frames:
// https://www.ros.org/reps/rep-0105.html

// OLD FILE. DO NOT RUN.

/*
FRAMES ATTACHED TO ROBOT:
base_footprint -- the bse of the robot at zero height above the ground
base_link -- the base of the robot

FRAMES ATTACHED TO ENVIRONMENT:
map -- the coordinate frame fixed to the map
odom -- what odometry thinks is its starting point

*/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){


    
    // this should be provided by SLAMh:
    // broadcaster.sendTransform(
    //   tf::StampedTransform(
    //     tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
    //     ros::Time::now(),"map", "base_link"));
    
    // this should come from the odometry
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(),"odom", "base_footprint"));


    r.sleep();
  }
}
