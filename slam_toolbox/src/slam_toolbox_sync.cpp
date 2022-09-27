/*
 * slam_toolbox
 * Copyright Work Modifications (c) 2018, Simbe Robotics, Inc.
 * Copyright Work Modifications (c) 2019, Steve Macenski
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#include "slam_toolbox/slam_toolbox_sync.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
SynchronousSlamToolbox::SynchronousSlamToolbox(ros::NodeHandle& nh)
: SlamToolbox(nh)
/*****************************************************************************/
{
  ssClear_ = nh.advertiseService("clear_queue",
    &SynchronousSlamToolbox::clearQueueCallback, this);

  threads_.push_back(std::make_unique<boost::thread>(
    boost::bind(&SynchronousSlamToolbox::run, this)));

  loadPoseGraphByParams(nh);
}

/*****************************************************************************/
void SynchronousSlamToolbox::run()
/*****************************************************************************/
{
  ros::Rate r(100);
  while(ros::ok())
  {
    if (!q_.empty() && !isPaused(PROCESSING))
    {
      PosedScan scan_w_pose = q_.front();
      q_.pop();

      if (q_.size() > 10)
      {
        ROS_WARN_THROTTLE(10., "Queue size has grown to: %i. "
          "Recommend stopping until message is gone if online mapping.",
          (int)q_.size());
      }

      karto::LocalizedRangeScan* karto_scan = addScan(getLaser(scan_w_pose.scan), scan_w_pose);
      addTag(scan_w_pose.apriltags, karto_scan);
      continue;
    }

    r.sleep();
  }
}

/*****************************************************************************/
void SynchronousSlamToolbox::laserCallback(
  const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  // no odom info on any pose helper
  karto::Pose2 pose;
  bool found_odom = false;
  for(size_t idx = 0; idx < pose_helpers_.size(); idx++)
  {
    found_odom = pose_helpers_[idx]->getOdomPose(pose, scan->header.stamp, scan->header.frame_id);
    if(found_odom)
      break;
  }
  if(!found_odom)
    return;

  // ensure the laser can be used
  karto::LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN_THROTTLE(5., "SynchronousSlamToolbox: Failed to create laser"
      " device for %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  std::stringstream ss(scan->header.frame_id);
  std::string frame_id;
  std::getline(ss, frame_id, '/');
  boost::mutex::scoped_lock lock(apriltag_q_mutex_);
  should_process = shouldProcessScan(scan, pose);

  // if sync and valid, add to queue
  if (should_process)
  {
    // find if apriltag was detected for agent, assumes laser_frame = apriltag_frame
    if (apriltags_q_.find(frame_id) != apriltags_q_.end() && !apriltags_q_[frame_id].empty()) {
      ROS_INFO("frame: %s", frame_id.c_str());
      q_.push(PosedScan(scan, pose, apriltags_q_[frame_id].front()));
      apriltags_q_[frame_id].pop();
    } else {
      q_.push(PosedScan(scan, pose));
    }
  }

  return;
}

/*****************************************************************************/
void SynchronousSlamToolbox::apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& apriltags) {
/*****************************************************************************/
  if (should_process) {
    std::stringstream ss(apriltags->header.frame_id);
    std::string frame_id;
    std::getline(ss, frame_id, '/');
    boost::mutex::scoped_lock lock(apriltag_q_mutex_);
    if (apriltags_q_.find(frame_id) == apriltags_q_.end())
      apriltags_q_[frame_id] = std::queue<apriltag_ros::AprilTagDetectionArray::ConstPtr>();
    apriltags_q_[frame_id].push(apriltags);
    ROS_INFO("Apriltag %d received in frame %s", apriltags->detections[0].id[0], frame_id.c_str());
  }
}

/*****************************************************************************/
bool SynchronousSlamToolbox::clearQueueCallback(
  slam_toolbox_msgs::ClearQueue::Request& req,
  slam_toolbox_msgs::ClearQueue::Response& resp)
/*****************************************************************************/
{
  ROS_INFO("SynchronousSlamToolbox: Clearing all queued scans to add to map.");
  while(!q_.empty())
  {
    q_.pop();
  }
  resp.status = true;
  return true;
}

/*****************************************************************************/
bool SynchronousSlamToolbox::deserializePoseGraphCallback(
  slam_toolbox_msgs::DeserializePoseGraph::Request& req,
  slam_toolbox_msgs::DeserializePoseGraph::Response& resp)
/*****************************************************************************/
{
  if (req.match_type == procType::LOCALIZE_AT_POSE)
  {
    ROS_ERROR("Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }
  return SlamToolbox::deserializePoseGraphCallback(req, resp);
}

} // end namespace
