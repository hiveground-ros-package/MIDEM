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


#include <midem_user_interaction/midem_user_interaction.h>
#include "ui_user_interaction.h"
#include <QtGui>
#include <tf/tf.h>


MidemUserInteraction::MidemUserInteraction(QMainWindow *parent) :
  QMainWindow(parent),
  ui(new Ui::MidemUserInteraction),
  quit_thread_(false),
  nhp_("~"),
  working_frame_("/base_link"),
  gesture_detector_loader_("midem_user_interaction","hg_gesture_detector::GestureDetector")
{
  ui->setupUi(this);
  dynamic_reconfigure::Server<midem_user_interaction::MidemUserInteractionConfig>::CallbackType f;
  f = boost::bind(&MidemUserInteraction::callbackConfig, this, _1, _2);
  reconfigure_server_.setCallback(f);

  arms_sub_ = nh_.subscribe("arms_msg", 1, &MidemUserInteraction::callbackArms, this);
  arm_gesture_pub_ = nhp_.advertise<interaction_msgs::Gestures>("arm_gestures", 1);

  skeletons_sub_ = nh_.subscribe("skeletons_msg", 1, &MidemUserInteraction::callbackSkeletons, this);
  skeleton_gesture_pub_ = nhp_.advertise<interaction_msgs::Gestures>("skeleton_gestures", 1);

  arms_syn_sub_.subscribe(nh_, "arms_msg", 10);
  skeletons_syn_sub_.subscribe(nh_, "skeletons_msg", 10);
  arm_skeleton_sync_.reset(new message_filters::Synchronizer<ArmBodyAppoxSyncPolicy>(ArmBodyAppoxSyncPolicy(10), arms_syn_sub_, skeletons_syn_sub_));
  arm_skeleton_sync_->registerCallback(boost::bind(&MidemUserInteraction::callbackArmsSkelentons, this, _1, _2));

  arm_gesture_markers_pub_ = nhp_.advertise<visualization_msgs::MarkerArray>("arm_gesture_markers", 1);
  skeleton_gesture_markers_pub_ = nhp_.advertise<visualization_msgs::MarkerArray>("skeleton_gesture_markers", 1);

  XmlRpc::XmlRpcValue gesture_detector;
  nhp_.getParam("gesture_detector", gesture_detector);
  if(gesture_detector.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR("invalid YAML structure");
    return;
  }

  ROS_INFO_STREAM("load " << gesture_detector.size() << " gesture detector(s)");
  for(XmlRpc::XmlRpcValue::iterator it = gesture_detector.begin(); it != gesture_detector.end(); it++)
  {
    ROS_INFO_STREAM("detector name: " << it->first);
    XmlRpc::XmlRpcValue detector;
    nhp_.getParam("gesture_detector/" + it->first, detector);
    if(detector.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("invalid YAML structure");
      return;
    }

    if(detector.begin()->first != "type")
    {
      ROS_ERROR("invalid YAML structure");
      return;
    }

    ROS_INFO_STREAM("         type: " << detector.begin()->second);
    try
    {
      gesture_detector_map_[it->first] = gesture_detector_loader_.createInstance(detector.begin()->second);
      gesture_detector_map_[it->first]->setName(it->first);
      if(!gesture_detector_map_[it->first]->initialize())
      {
        ROS_ERROR_STREAM("Cannot initialize detector " << it->first);
        gesture_detector_map_.erase(it->first);
      }
    }
    catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

  }
}

MidemUserInteraction::~MidemUserInteraction()
{
  delete ui;
}

void MidemUserInteraction::callbackConfig(midem_user_interaction::MidemUserInteractionConfig &config, uint32_t level)
{
  ROS_INFO("Get config: %f", config.min_x);
}

void MidemUserInteraction::callbackArms(const interaction_msgs::ArmsConstPtr &msg)
{
  interaction_msgs::ArmsPtr arm_msgs(new interaction_msgs::Arms);
  *arm_msgs = *msg;

  if(arm_msgs->header.frame_id != working_frame_)
  {
    tf::StampedTransform stf;
    try
    {
      tf_listener_.lookupTransform(working_frame_, arm_msgs->header.frame_id, ros::Time(0), stf);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s",ex.what());
      return;
    }

    //transform position    
    tf::Transform tf;
    for(size_t i = 0; i < arm_msgs->arms.size(); i++)
    {
      tf::transformMsgToTF(arm_msgs->arms[i].hand, tf);
      tf = stf * tf;
      tf::transformTFToMsg(tf, arm_msgs->arms[i].hand);
    }

    //update frame name
    arm_msgs->header.frame_id = working_frame_;
  }

  interaction_msgs::Gestures::Ptr gestures_msg(new interaction_msgs::Gestures);
  visualization_msgs::MarkerArray marker_array;
  GestureDetectorMap::iterator it;
  for(it = gesture_detector_map_.begin(); it != gesture_detector_map_.end(); it++)
  {
    boost::shared_ptr<hg_gesture_detector::HandGestureDetector> hand_detector =
        boost::dynamic_pointer_cast<hg_gesture_detector::HandGestureDetector>(it->second);
    if(hand_detector)
    {
      hand_detector->addMessage(arm_msgs);
      hand_detector->lookForGesture(*gestures_msg);
      hand_detector->getMarkers(marker_array, arm_msgs->header.frame_id);
    }
  }

  if(arm_gesture_pub_.getNumSubscribers() != 0 && gestures_msg->gestures.size() != 0)
  {
    gestures_msg->header.stamp = ros::Time::now();
    gestures_msg->header.frame_id = arm_msgs->header.frame_id;
    arm_gesture_pub_.publish(gestures_msg);
  }


  if(arm_gesture_markers_pub_.getNumSubscribers() != 0 && marker_array.markers.size() != 0)
  {
    arm_gesture_markers_pub_.publish(marker_array);
  }
}

void MidemUserInteraction::callbackSkeletons(const kinect_msgs::SkeletonsConstPtr& msg)
{ 
  kinect_msgs::SkeletonsPtr skeletons_msgs(new kinect_msgs::Skeletons);
  *skeletons_msgs = *msg;
  if(skeletons_msgs->header.frame_id != working_frame_)
  {
    tf::StampedTransform stf;
    try
    {
      tf_listener_.lookupTransform(working_frame_, skeletons_msgs->header.frame_id, ros::Time(0), stf);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s",ex.what());
      return;
    }

    tf::Transform tf;    
    for(size_t i = 0; i < kinect_msgs::Skeletons::SKELETON_COUNT; i++)
    {
      if(skeletons_msgs->skeletons[i].skeleton_tracking_state == kinect_msgs::Skeleton::SKELETON_TRACKED)
      {
        for(size_t j = 0; j < kinect_msgs::Skeleton::SKELETON_POSITION_COUNT; j++)
        {
          //msg->skeletons[i].skeleton_positions[j]
          tf::transformMsgToTF(skeletons_msgs->skeletons[i].skeleton_positions[j], tf);
          tf = stf * tf;
          tf::transformTFToMsg(tf, skeletons_msgs->skeletons[i].skeleton_positions[j]);
        }

        tf::transformMsgToTF(skeletons_msgs->skeletons[i].position, tf);
        tf = stf * tf;
        tf::transformTFToMsg(tf, skeletons_msgs->skeletons[i].position);
      }
    }

    //update frame name
    skeletons_msgs->header.frame_id = working_frame_;

  }

  std_msgs::ColorRGBA color_joint, color_link;
  color_joint.r = 1.0; color_joint.g = 0.0; color_joint.b = 0.0; color_joint.a = 0.5;
  color_link.r = 0.0; color_link.g = 1.0; color_link.b = 0.0; color_link.a = 0.5;
  visualization_msgs::MarkerArray marker_array;
  int marker_count = 0;
  GestureDetectorMap::iterator it;
  interaction_msgs::Gestures::Ptr gestures_msg(new interaction_msgs::Gestures);
  for(size_t i = 0; i < kinect_msgs::Skeletons::SKELETON_COUNT; i++)
  {
    if(skeletons_msgs->skeletons[i].skeleton_tracking_state == kinect_msgs::Skeleton::SKELETON_TRACKED)
    {

      for(it = gesture_detector_map_.begin(); it != gesture_detector_map_.end(); it++)
      {
        boost::shared_ptr<hg_gesture_detector::SkelentonGestureDetector> skeleton_detector =
            boost::dynamic_pointer_cast<hg_gesture_detector::SkelentonGestureDetector>(it->second);
        if(skeleton_detector)
        {
          skeleton_detector->addMessage(skeletons_msgs->skeletons[i]);
          skeleton_detector->lookForGesture(*gestures_msg);
          skeleton_detector->getMarkers(marker_array, skeletons_msgs->header.frame_id);
        }
      }

      if(skeleton_gesture_markers_pub_.getNumSubscribers() != 0)
      {
        getSkeletonMarker(skeletons_msgs->skeletons[i],
                          skeletons_msgs->header.frame_id,
                          color_joint, color_link,
                          marker_array, marker_count);
      }

    }
  }

  if(skeleton_gesture_pub_.getNumSubscribers() != 0 && gestures_msg->gestures.size() != 0)
  {
    gestures_msg->header.stamp = ros::Time::now();
    gestures_msg->header.frame_id = skeletons_msgs->header.frame_id;
    skeleton_gesture_pub_.publish(gestures_msg);
  }

  if(skeleton_gesture_markers_pub_.getNumSubscribers() != 0 && marker_array.markers.size() != 0)
  {
    skeleton_gesture_markers_pub_.publish(marker_array);
  }

}

void MidemUserInteraction::callbackArmsSkelentons(const interaction_msgs::ArmsConstPtr& arms_msg,
                                                  const kinect_msgs::SkeletonsConstPtr& skelentons_msg)
{
   //ROS_INFO("haho");
}

void MidemUserInteraction::getSkeletonMarker(const kinect_msgs::Skeleton& skeleton,
                                             const std::string& frame_id,
                                             const std_msgs::ColorRGBA& color_joint,
                                             const std_msgs::ColorRGBA& color_link,
                                             visualization_msgs::MarkerArray& marker_array,
                                             int& last_marker_id)
{
  using namespace kinect_msgs;
  using namespace visualization_msgs;

  Marker links;
  Marker joints;
  joints.type = Marker::SPHERE_LIST;
  joints.lifetime = ros::Duration(0.1);
  joints.ns = "KinectSkeleton";
  joints.header.frame_id = frame_id;
  joints.id = last_marker_id++;
  joints.scale.x = joints.scale.y = joints.scale.z = 0.05;
  joints.pose.position.x = 0;
  joints.pose.position.y = 0;
  joints.pose.position.z = 0;
  joints.pose.orientation.x = 0;
  joints.pose.orientation.y = 0;
  joints.pose.orientation.z = 0;
  joints.pose.orientation.w = 1;
  joints.color = color_joint;
  links =  joints;
  links.type = Marker::LINE_LIST;
  links.id = last_marker_id++;
  links.color = color_link;
  links.scale.x = links.scale.y = links.scale.z = 0.02;

  geometry_msgs::Point point;
  for(int i = 0; i < Skeleton::SKELETON_POSITION_COUNT; i++)
  {
    point.x = skeleton.skeleton_positions[i].translation.x;
    point.y = skeleton.skeleton_positions[i].translation.y;
    point.z = skeleton.skeleton_positions[i].translation.z;
    joints.points.push_back(point);
  }
  marker_array.markers.push_back(joints);


  //upper body
  int shoulder_center_state = skeleton.skeleton_position_tracking_state[Skeleton::SKELETON_POSITION_SHOULDER_CENTER];
  if(shoulder_center_state == Skeleton::SKELETON_POSITION_TRACKED ||  shoulder_center_state == Skeleton::SKELETON_POSITION_INFERRED)
  {
    //Head
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_CENTER]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_HEAD]);

    //Left
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_CENTER]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_LEFT]);

    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_LEFT]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_ELBOW_LEFT]);

    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_ELBOW_LEFT]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_WRIST_LEFT]);

    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_WRIST_LEFT]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_HAND_LEFT]);

    //Right
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_CENTER]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_RIGHT]);

    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_SHOULDER_RIGHT]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_ELBOW_RIGHT]);

    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_ELBOW_RIGHT]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_WRIST_RIGHT]);

    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_WRIST_RIGHT]);
    links.points.push_back(joints.points[Skeleton::SKELETON_POSITION_HAND_RIGHT]);
  }

  //lower body

  marker_array.markers.push_back(links);
}

void MidemUserInteraction::closeEvent(QCloseEvent *event)
{
  quit_thread_ = true;
  event->accept();
}


