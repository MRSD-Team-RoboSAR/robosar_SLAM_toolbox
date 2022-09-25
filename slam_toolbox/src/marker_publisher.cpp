#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"

// Agent info
const std::vector<std::string> agent_names = {"agent1", "agent2"};
// Topic info
const std::string topic_name_prefix = "/robosar_agent_bringup_node/";
const std::string topic_name_suffix = "/feedback/apriltag";

ros::Publisher msg_pub;
std::map<int, std_msgs::ColorRGBA> tag_color_map;

void msgCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  for(size_t det_idx = 0; det_idx < msg->detections.size(); det_idx++)
  {
    for(size_t id_idx = 0; id_idx < msg->detections[det_idx].id.size(); id_idx++)
    {
      visualization_msgs::Marker m;
      m.header.frame_id = msg->header.frame_id;
      // label
      m.ns = msg->header.frame_id;
      m.id = msg->detections[det_idx].id[id_idx];
      // shape
      m.type = 2; // Sphere
      // what to do
      m.action = 0; // Add/modify
      // pose
      m.pose.position.x = msg->detections[det_idx].pose.pose.pose.position.x;
      m.pose.position.y = msg->detections[det_idx].pose.pose.pose.position.y;
      m.pose.position.z = msg->detections[det_idx].pose.pose.pose.position.z;
      m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0;
      m.pose.orientation.w = 1;
      // size
      m.scale.x = 0.3;
      m.scale.y = 0.3;
      m.scale.z = 0.3;
      // color - use previous color or create new one
      if(tag_color_map.find(m.id) == tag_color_map.end())
      {
        // New tag; create and save color
        std_msgs::ColorRGBA new_color;
        new_color.r = (float)(rand() % 100) / 100;
        new_color.g = (float)(rand() % 100) / 100;
        new_color.b = (float)(rand() % 100) / 100;
        new_color.a = 1;
        m.color = new_color;
        tag_color_map[m.id] = new_color;
      }
      else
      {
        // Old tag; reuse color
        m.color = tag_color_map[m.id];
      }
      // lifetime
      m.lifetime = ros::Duration(0); // forever
      // publish
      msg_pub.publish(m);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle nh;
  std::vector<ros::Subscriber> msg_subscribers;
  for(size_t agent_idx = 0; agent_idx < agent_names.size(); agent_idx++){
    const std::string topic_name = topic_name_prefix + agent_names.at(agent_idx) + topic_name_suffix;
    msg_subscribers.push_back(nh.subscribe(topic_name, 1000, msgCallback));
  }
  msg_pub = nh.advertise<visualization_msgs::Marker>("victim_markers", 1000);
  ros::spin();
  return 0;
}
