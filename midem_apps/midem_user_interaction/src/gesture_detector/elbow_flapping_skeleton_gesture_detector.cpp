/*
 * Copyright (c) 2013, HiveGround Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the HiveGround Co., Ltd., nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 */



#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

#include <gesture_detector/gesture_detector_utils.h>
#include <gesture_detector/elbow_flapping_skeleton_gesture_detector.h>

PLUGINLIB_EXPORT_CLASS(hg_gesture_detector::ElbowFlappingSkeletonGestureDetector,hg_gesture_detector::GestureDetector)

using namespace hg_gesture_detector;
using namespace kinect_msgs;
using namespace visualization_msgs;


ElbowFlappingSkeletonGestureDetector::ElbowFlappingSkeletonGestureDetector()
  : min_flapping_length_(0.1),
    detect_rate_(1.0)
{

}

bool ElbowFlappingSkeletonGestureDetector::initialize()
{
  return true;
}

void ElbowFlappingSkeletonGestureDetector::addMessage(const kinect_msgs::Skeleton& msg)
{
  if(msg.skeleton_tracking_state != Skeleton::SKELETON_TRACKED)
    return;

  tf::Transform left_tf, right_tf;
  tf::transformMsgToTF(msg.skeleton_positions[Skeleton::SKELETON_POSITION_ELBOW_LEFT], left_tf);
  tf::transformMsgToTF(msg.skeleton_positions[Skeleton::SKELETON_POSITION_ELBOW_RIGHT], right_tf);
  left_elbow_path_.push_back(left_tf);
  right_elbow_path_.push_back(right_tf);
  if(left_elbow_path_.size() > MAX_ELBOW_PATH) left_elbow_path_.pop_front();
  if(right_elbow_path_.size() > MAX_ELBOW_PATH) right_elbow_path_.pop_front();
  skeleton_id_ = msg.tracking_id;
}

void ElbowFlappingSkeletonGestureDetector::getMarkers(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id)
{
  Marker marker;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = frame_id;
  marker.ns = "ElbowFlappingSkeletonGestureDetector";
  marker.type = Marker::LINE_STRIP;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.id = 0;

  std::list<tf::Transform>::iterator it;
  int idx = 0;
  if(left_elbow_path_.size() > 1)
  {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.points.resize(left_elbow_path_.size());
    for(it = left_elbow_path_.begin(), idx = 0; it != left_elbow_path_.end(); it++, idx++)
    {
      marker.points[idx].x = it->getOrigin().x();
      marker.points[idx].y = it->getOrigin().y();
      marker.points[idx].z = it->getOrigin().z();
    }
    marker_array.markers.push_back(marker);
  }


  marker.id++;
  if(right_elbow_path_.size() > 1)
  {
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.points.resize(right_elbow_path_.size());
    for (it = right_elbow_path_.begin(), idx = 0; it != right_elbow_path_.end(); it++, idx++)
    {
      marker.points[idx].x = it->getOrigin().x();
      marker.points[idx].y = it->getOrigin().y();
      marker.points[idx].z = it->getOrigin().z();
    }
    marker_array.markers.push_back(marker);
  }
}

void ElbowFlappingSkeletonGestureDetector::lookForGesture(interaction_msgs::Gestures& gestures)
{
  //check length
  double l_length, r_length;
  l_length = r_length = 0;
  tf::Vector3 l_dir, r_dir;
  if(left_elbow_path_.size() > 10)
  {
    l_dir = left_elbow_path_.back().getOrigin() - left_elbow_path_.front().getOrigin();
    l_length = l_dir.length();
  }

  if(right_elbow_path_.size() > 10)
  {
    r_dir = right_elbow_path_.back().getOrigin() - right_elbow_path_.front().getOrigin();
    r_length = r_dir.length();
  }

  if((l_length > min_flapping_length_) && (r_length > min_flapping_length_))
  {
    double dt = (ros::Time::now() - last_detected_time_).toSec();
    if(dt >= 1/detect_rate_)
    {
      //check direction
      if((l_dir.z() > 0) && (r_dir.z() > 0))
      {
        if((l_dir.z() > l_dir.x()) &&
           (l_dir.z() > l_dir.y()) &&
           (r_dir.z() > r_dir.x()) &&
           (r_dir.z() > r_dir.y()))
        {
          left_elbow_path_.clear();
          right_elbow_path_.clear();
          last_detected_time_ = ros::Time::now();
          interaction_msgs::Gesture gesture;
          std::stringstream ss;
          ss << "ELBOW_FLAPPING_SKELETON_" << skeleton_id_;
          gesture.name = ss.str();
          gestures.gestures.push_back(gesture);
          ROS_DEBUG("l_length: %6.3f r_length: %6.3f", l_length, r_length);
        }
      }
    }
  }
}


