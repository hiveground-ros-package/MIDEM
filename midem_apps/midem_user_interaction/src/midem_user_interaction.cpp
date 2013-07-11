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
  working_frame_("base_link"),
  gesture_detector_loader_("midem_user_interaction","hg_gesture_detector::GestureDetector")
{
  ui->setupUi(this);
  dynamic_reconfigure::Server<midem_user_interaction::MidemUserInteractionConfig>::CallbackType f;
  f = boost::bind(&MidemUserInteraction::callbackConfig, this, _1, _2);
  reconfigure_server_.setCallback(f);

  arms_sub_ = nh_.subscribe("arms_msg", 1, &MidemUserInteraction::callbackArms, this);
  arm_joy_pub_ = nhp_.advertise<sensor_msgs::JointState>("hands_joy", 1);

  skeletons_sub_ = nh_.subscribe("skeletons_msg", 1, &MidemUserInteraction::callbackSkeletons, this);
  skeleton_joy_pub_ = nhp_.advertise<sensor_msgs::JointState>("body_joy", 1);

  arms_syn_sub_.subscribe(nh_, "arms_msg", 10);
  skeletons_syn_sub_.subscribe(nh_, "skeletons_msg", 10);
  arm_skeleton_sync_.reset(new message_filters::Synchronizer<ArmBodyAppoxSyncPolicy>(ArmBodyAppoxSyncPolicy(10), arms_syn_sub_, skeletons_syn_sub_));
  arm_skeleton_sync_->registerCallback(boost::bind(&MidemUserInteraction::callbackArmsSkelentons, this, _1, _2));

  markers_pub_ = nhp_.advertise<visualization_msgs::MarkerArray>("markers", 1);



  boost::shared_ptr<hg_gesture_detector::GestureDetector> p;
  try
  {
    p = gesture_detector_loader_.createInstance("hg_gesture_detector::RubberBandHandGestureDetector");
    p->addMarker(marker_array_);
    interaction_msgs::Gestures gestures;
    p->lookForGesture(gestures);

    boost::shared_ptr<hg_gesture_detector::HandGestureDetector> hand_detector = boost::dynamic_pointer_cast<hg_gesture_detector::HandGestureDetector>(p);
    if(hand_detector)
    {
      ROS_INFO("hand");
    }
    else
    {
      ROS_INFO("convert to hand failed!");
    }

    boost::shared_ptr<hg_gesture_detector::SkelentonGestureDetector> skeleton_detector = boost::dynamic_pointer_cast<hg_gesture_detector::SkelentonGestureDetector>(p);
    if(skeleton_detector)
    {
      ROS_INFO("skeleton");
    }
    else
    {
      ROS_INFO("convert to skeletion failed!");
    }

  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
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
  if(msg->arms.size() == 0)
    return;
  std::vector<tf::Transform> hand_transforms(msg->arms.size());
  for(size_t i = 0; i < i < msg->arms.size(); i++)
  {
    tf::transformMsgToTF(msg->arms[i].hand, hand_transforms[i]);
  }

  //ROS_INFO_THROTTLE(1.0, "Arm %s %s", msg->header.frame_id.c_str(), working_frame_.c_str());
  if(msg->header.frame_id != working_frame_)
  {

    tf::StampedTransform stf;
    try
    {
      tf_listener_.lookupTransform("base_link", msg->header.frame_id, ros::Time::now(), stf);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return;
    }

    //transform position
    for(size_t i = 0; i < i < hand_transforms.size(); i++)
    {
      hand_transforms[i] = hand_transforms[i] * stf;
    }
  }




}

void MidemUserInteraction::callbackSkeletons(const kinect_msgs::SkeletonsConstPtr& msg)
{
  //ROS_INFO_THROTTLE(1.0, "hoho");
}

void MidemUserInteraction::callbackArmsSkelentons(const interaction_msgs::ArmsConstPtr& arms_msg,
                                                  const kinect_msgs::SkeletonsConstPtr& skelentons_msg)
{
   //ROS_INFO("haho");
}

void MidemUserInteraction::closeEvent(QCloseEvent *event)
{
  quit_thread_ = true;
  event->accept();
}


