#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/String.h"
#include <string>

ros::Publisher msg_pub;

void msgCallback(const std_msgs::String::ConstPtr& msg)
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
  m.pose.position.x = std::stof(msg->data);
  m.pose.position.y = 1;
  m.pose.position.z = 0.5;
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
