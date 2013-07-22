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
 *
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

#include <gesture_detector/gesture_detector_utils.h>
#include <gesture_detector/rubber_band_skeleton_gesture_detector.h>

PLUGINLIB_EXPORT_CLASS(hg_gesture_detector::RubberBandSkeletonGestureDetector,hg_gesture_detector::GestureDetector)

using namespace hg_gesture_detector;
using namespace kinect_msgs;
using namespace visualization_msgs;

const std::string RubberBandSkeletonGestureDetector::SKELETON_GESTURE = "RUBBER_BAND_SKELETON_";

RubberBandSkeletonGestureDetector::RubberBandSkeletonGestureDetector()
  : state_(IDEL),
    r_activating_(0.02), r_activated_(0.03), r_leaving_(0.08), r_die_(0.1),
    activating_time_(2.0)
{

}

bool RubberBandSkeletonGestureDetector::initialize()
{
  return true;
}

void RubberBandSkeletonGestureDetector::addMessage(const kinect_msgs::Skeleton& msg)
{
  if(msg.skeleton_tracking_state != Skeleton::SKELETON_TRACKED)
    return;

  //use left and right shoulder for gesture articulating
  tf::Transform left_tf, right_tf;
  tf::transformMsgToTF(msg.skeleton_positions[Skeleton::SKELETON_POSITION_SHOULDER_LEFT], left_tf);
  tf::transformMsgToTF(msg.skeleton_positions[Skeleton::SKELETON_POSITION_SHOULDER_RIGHT], right_tf);
  last_skeleton_position_ = (left_tf.getOrigin() + right_tf.getOrigin()) * 0.5;
  skeleton_id_ = msg.tracking_id;
}

void RubberBandSkeletonGestureDetector::getMarkers(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id)
{
  Marker marker;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = frame_id;
  marker.ns = "RubberBandSkeletonGestureDetector";
  marker.type = Marker::SPHERE;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.id = 0;

  //currrent position
  marker.pose.position.x = last_skeleton_position_.getX();
  marker.pose.position.y = last_skeleton_position_.getY();
  marker.pose.position.z = last_skeleton_position_.getZ();
  marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.id++;
  marker_array.markers.push_back(marker);

  switch(state_)
  {
    case IDEL:
      marker.pose.position.x = last_skeleton_position_.getX();
      marker.pose.position.y = last_skeleton_position_.getY();
      marker.pose.position.z = last_skeleton_position_.getZ();
      break;
    case ACTIVATING:
    case ACTIVATED:
    case MOVING:
    case LEAVING:
      marker.pose.position.x = pivot_position_.getX();
      marker.pose.position.y = pivot_position_.getY();
      marker.pose.position.z = pivot_position_.getZ();
      break;
  }

  //r1
  marker.scale.x = marker.scale.y = marker.scale.z = (state_ <= ACTIVATING) ? r_activating_*2 : r_activated_*2;
  marker.color.r = (state_ == ACTIVATED ? 0.0 : (state_ == ACTIVATING ? 0.5 : 1.0));
  marker.color.g = (state_ == MOVING ? 0.5 : (state_ == ACTIVATED ? 1.0 : 0.0));
  marker.color.b = 0.0;
  marker.color.a = 0.5;
  marker.id++;
  marker_array.markers.push_back(marker);

  //R2
  marker.scale.x = marker.scale.y = marker.scale.z = r_leaving_ * 2;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.2;
  marker.id++;
  marker_array.markers.push_back(marker);

  //R3
  marker.scale.x = marker.scale.y = marker.scale.z = r_die_ * 2;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 0.1;
  marker.id++;
  marker_array.markers.push_back(marker);


}

void RubberBandSkeletonGestureDetector::lookForGesture(interaction_msgs::Gestures& gestures)
{
  tf::Vector3 vec_to_skeleton = last_skeleton_position_ - pivot_position_;
  rubberBandStateTransition(state_,
                            pivot_position_,
                            last_skeleton_position_,
                            start_activating_time_,
                            activating_time_,
                            r_activating_,
                            r_activated_,
                            r_leaving_,
                            r_die_);
  if(state_ == MOVING)
  {
    std::stringstream ss;
    interaction_msgs::Gesture gesture;
    switch(checkDirection(vec_to_skeleton))
    {
      case UP: ss << SKELETON_GESTURE << skeleton_id_ << "_UP"; break;
      case DOWN: ss << SKELETON_GESTURE << skeleton_id_ << "_DOWN"; break;
      case LEFT: ss << SKELETON_GESTURE << skeleton_id_ << "_LEFT"; break;
      case RIGHT: ss << SKELETON_GESTURE << skeleton_id_ << "_RIGHT"; break;
      case FORWARD: ss << SKELETON_GESTURE << skeleton_id_ << "_FORWARD"; break;
      case BACKWARD: ss << SKELETON_GESTURE << skeleton_id_ << "_BACKWARD"; break;
    }
    ROS_DEBUG_STREAM(ss.str());
    gesture.name = ss.str();
    gestures.gestures.push_back(gesture);
  }
}


