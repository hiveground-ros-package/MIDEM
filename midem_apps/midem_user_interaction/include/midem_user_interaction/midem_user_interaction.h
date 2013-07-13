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


#ifndef MIDEM_USER_INTERACTION_H
#define MIDEM_USER_INTERACTION_H

#include <QMainWindow>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <midem_user_interaction/midem_user_interaction.h>
#include <midem_user_interaction/MidemUserInteractionConfig.h>

#include <sensor_msgs/JointState.h>

#include <interaction_msgs/Arms.h>
#include <kinect_msgs/Skeletons.h>

#include <visualization_msgs/MarkerArray.h>

#include <pluginlib/class_loader.h>
#include <gesture_detector/gesture_detector.h>
#include <gesture_detector/hand_gesture_detector.h>
#include <gesture_detector/skeleton_gesture_detector.h>

namespace Ui {
  class MidemUserInteraction;
}

class MidemUserInteraction : public QMainWindow
{
  Q_OBJECT

public:
  explicit MidemUserInteraction(QMainWindow *parent = 0);
  ~MidemUserInteraction();
  bool isQuit() { return quit_thread_; }

protected Q_SLOTS:
  void test() { }

protected: //Local
  void callbackConfig(midem_user_interaction::MidemUserInteractionConfig &config, uint32_t level);
  void callbackArms(const interaction_msgs::ArmsConstPtr& msg);
  void callbackSkeletons(const kinect_msgs::SkeletonsConstPtr& msg);
  void callbackArmsSkelentons(const interaction_msgs::ArmsConstPtr& arms_msg,
                              const kinect_msgs::SkeletonsConstPtr& skelentons_msg);

protected: //Qt
  void closeEvent(QCloseEvent *evencurrentItemt);

private:
  Ui::MidemUserInteraction *ui;
  bool quit_thread_;
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;  
  std::string working_frame_;
  pluginlib::ClassLoader<hg_gesture_detector::GestureDetector> gesture_detector_loader_;

  ros::Publisher arm_gesture_markers_pub_;
  ros::Publisher arm_gesture_pub_;

  ros::Publisher skeleton_gesture_markers_pub_;
  ros::Publisher skeleton_gesture_pub_;

  tf::TransformListener tf_listener_;

  dynamic_reconfigure::Server<midem_user_interaction::MidemUserInteractionConfig> reconfigure_server_;
  ros::Subscriber arms_sub_;


  ros::Subscriber skeletons_sub_;


  message_filters::Subscriber<interaction_msgs::Arms> arms_syn_sub_;
  message_filters::Subscriber<kinect_msgs::Skeletons> skeletons_syn_sub_;
  typedef message_filters::sync_policies::ApproximateTime<interaction_msgs::Arms, kinect_msgs::Skeletons> ArmBodyAppoxSyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<ArmBodyAppoxSyncPolicy> > arm_skeleton_sync_;




  typedef std::map<std::string, boost::shared_ptr<hg_gesture_detector::GestureDetector> > GestureDetectorMap;
  GestureDetectorMap gesture_detector_map_;
};

#endif // MIDEM_USER_INTERACTION_H
