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

#include <gesture_detector/rubber_band_hand_gesture_detector.h>
#include <gesture_detector/gesture_detector_utils.h>

PLUGINLIB_EXPORT_CLASS(hg_gesture_detector::RubberBandHandGestureDetector,hg_gesture_detector::GestureDetector)

using namespace hg_gesture_detector;
using namespace visualization_msgs;

const std::string RubberBandHandGestureDetector::ONE_HANDED = "RUBBER_BAND_HAND_";
const std::string RubberBandHandGestureDetector::TWO_HANDED = "RUBBER_BAND_HANDS_";

RubberBandHandGestureDetector::RubberBandHandGestureDetector()
  : last_hand_count_(0),
    r_activating_(0.03), r_activated_(0.05), r_leaving_(0.2), r_die_(0.25),
    activating_time_(2.0)
{

}

bool RubberBandHandGestureDetector::initialize()
{
  return true;
}

void RubberBandHandGestureDetector::addMessage(const interaction_msgs::ArmsPtr& msg)
{
  if(msg->arms.size() == 0)
  {
    hand_states_.clear();
    return;
  }

  std::vector<tf::Vector3> hand_positions(msg->arms.size());
  for(size_t i = 0; i < msg->arms.size(); i++)
  {
    tf::vector3MsgToTF(msg->arms[i].hand.translation, hand_positions[i]);
  }

  if(msg->arms.size() != hand_states_.size())
  {
    //reset/initiaize
    hand_states_.clear();
    hand_states_.resize(msg->arms.size());
    for(size_t i = 0; i < msg->arms.size(); i++)
    {
      hand_states_[i].last_hand_position = hand_positions[i];
    }
  }
  else
  {
    //update each hand state with the closet hand
    int index;
    for(size_t i = 0; i < hand_states_.size(); i++)
    {
      index = closetHandIndex(hand_states_[i].last_hand_position, hand_positions);
      hand_states_[i].last_hand_position = hand_positions[index];      
      hand_positions.erase(hand_positions.begin() + index);
    }
  }
}

void RubberBandHandGestureDetector::getMarkers(visualization_msgs::MarkerArray& marker_array,
                                               const std::string& frame_id)
{
  if(hand_states_.size() == 0)
    return;

  Marker marker;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = frame_id;
  marker.ns = "RubberBandHandGestureDetector";
  marker.type = Marker::SPHERE;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.id = 0;

  for(size_t i = 0; i < hand_states_.size(); i++)
  {
    switch(hand_states_[i].state)
    {
      case IDEL:
        marker.pose.position.x = hand_states_[i].last_hand_position.getX();
        marker.pose.position.y = hand_states_[i].last_hand_position.getY();
        marker.pose.position.z = hand_states_[i].last_hand_position.getZ();
        break;      
      case ACTIVATING:        
      case ACTIVATED:              
      case MOVING:
      case LEAVING:
        marker.pose.position.x = hand_states_[i].pivot_position.getX();
        marker.pose.position.y = hand_states_[i].pivot_position.getY();
        marker.pose.position.z = hand_states_[i].pivot_position.getZ();
        break;
    }

    //r1
    marker.scale.x = marker.scale.y = marker.scale.z = (hand_states_[i].state <= ACTIVATING) ? r_activating_*2 : r_activated_*2;
    marker.color.r = (hand_states_[i].state == ACTIVATED ? 0.0 : (hand_states_[i].state == ACTIVATING ? 0.5 : 1.0));
    marker.color.g = (hand_states_[i].state == MOVING ? 0.5 : (hand_states_[i].state == ACTIVATED ? 1.0 : 0.0));
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
}

void RubberBandHandGestureDetector::lookForGesture(interaction_msgs::Gestures& gestures)
{
  std::vector<int> directions(hand_states_.size(), UNKNOWN);
  for(size_t i = 0; i < hand_states_.size(); i++)
  {
    tf::Vector3 vec_to_hand = hand_states_[i].last_hand_position - hand_states_[i].pivot_position;
    rubberBandStateTransition(hand_states_[i].state,
                              hand_states_[i].pivot_position,
                              hand_states_[i].last_hand_position,
                              hand_states_[i].start_activating_time,
                              activating_time_,
                              r_activating_,
                              r_activated_,
                              r_leaving_,
                              r_die_);
    if(hand_states_[i].state == MOVING)
    {
      directions[i] = checkDirection(vec_to_hand);
    }
  }

  if(hand_states_.size() == 1)
  {
    interaction_msgs::Gesture gesture;
    std::stringstream ss;
    switch(directions[0])
    {
      case UP: ss << ONE_HANDED << "UP"; break;
      case DOWN: ss << ONE_HANDED << "DOWN"; break;
      case LEFT: ss << ONE_HANDED << "LEFT"; break;
      case RIGHT: ss << ONE_HANDED << "RIGHT"; break;
      case FORWARD: ss << ONE_HANDED << "FORWARD"; break;
      case BACKWARD: ss << ONE_HANDED << "BACKWARD"; break;
    }
    if(ss.str().length() != 0)
    {
      gesture.name = ss.str();
      gestures.gestures.push_back(gesture);
    }
  }


  if(hand_states_.size() == 2)
  {
    int dir_left, dir_right;
    if(hand_states_[0].pivot_position.getY() < hand_states_[1].pivot_position.getY())
    {
      dir_left = directions[0]; dir_right = directions[1];
    }
    else
    {
      dir_left = directions[1]; dir_right = directions[0];
    }
    interaction_msgs::Gesture gesture;
    switch(dir_left)
    {
      case UP:
        if(dir_right == DOWN)
          gesture.name = TWO_HANDED + "ROTATE_X-";
        break;
      case DOWN:
        if(dir_right == UP)
          gesture.name = TWO_HANDED + "ROTATE_X+";
        break;
      case FORWARD:
        if(dir_right == FORWARD)
          gesture.name = TWO_HANDED + "ROTATE_Y+";
        else if(dir_right == BACKWARD)
          gesture.name = TWO_HANDED + "ROTATE_Z+";
        break;
      case BACKWARD:
        if(dir_right == FORWARD)
          gesture.name = TWO_HANDED + "ROTATE_Z-";
        else if(dir_right == BACKWARD)
          gesture.name = TWO_HANDED + "ROTATE_Y-";
        break;
    }

    if(gesture.name.length() != 0)
    {
      ROS_DEBUG_STREAM(gesture.name);
      gestures.gestures.push_back(gesture);
    }
  }
}

int RubberBandHandGestureDetector::closetHandIndex(const tf::Vector3 point,
                                                   const std::vector<tf::Vector3>& hand_positions)
{
  double distance, min_distance = 1e6;
  size_t index = -1;
  for (size_t i = 0; i < hand_positions.size(); i++)
  {
    distance = point.distance(hand_positions[i]);
    if (distance < min_distance)
    {
      min_distance = distance;
      index = i;
    }
  }
  return index;
}
