#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"

ros::Publisher msg_pub;

void msgCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  visualization_msgs::Marker m;
  m.header.frame_id = "map";
  // label
  m.ns = "my_namespace";
  m.id = 1234321;
  // shape
  m.type = 2; // Sphere
  // what to do
  m.action = 0; // Add/modify
  // pose
  m.pose.position.x = msg->detections[0].pose.pose.pose.position.x;
  m.pose.position.y = msg->detections[0].pose.pose.pose.position.y;
  m.pose.position.z = msg->detections[0].pose.pose.pose.position.z;
  m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0;
  m.pose.orientation.w = 1;
  // size
  m.scale.x = 0.3;
  m.scale.y = 0.3;
  m.scale.z = 0.3;
  // color
  m.color.r = 0.2;
  m.color.g = 0.2;
  m.color.b = 1;
  m.color.a = 1;
  // lifetime
  m.lifetime = ros::Duration(0); // forever
  // publish
  msg_pub.publish(m);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  ros::Subscriber msg_sub = nh.subscribe("topic_name", 1000, msgCallback);
  msg_pub = nh.advertise<visualization_msgs::Marker>("chatter", 1000);
  ros::spin();
  return 0;
}
