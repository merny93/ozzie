#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(5.0, 1.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 1.0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cmd_vel_tf");

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/cmd_vel", 10, &poseCallback);

  ros::spin();
  return 0;
};