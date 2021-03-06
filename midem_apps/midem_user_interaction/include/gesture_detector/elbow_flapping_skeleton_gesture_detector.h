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

#ifndef ELBOW_FLAPPING_SKELETON_GESTURE_DETECTOR_H_
#define ELBOW_FLAPPING_SKELETON_GESTURE_DETECTOR_H_

#include <tf/tf.h>
#include "skeleton_gesture_detector.h"


namespace hg_gesture_detector
{

class ElbowFlappingSkeletonGestureDetector : public SkelentonGestureDetector
{
  static const int MAX_ELBOW_PATH = 20;

public:
  static const std::string SKELETON_GESTURE;

  ElbowFlappingSkeletonGestureDetector();
  virtual ~ElbowFlappingSkeletonGestureDetector() { }

  bool initialize();
  void addMessage(const kinect_msgs::Skeleton& msg);
  void getMarkers(visualization_msgs::MarkerArray& marker_array, const std::string& frame_id);
  void lookForGesture(interaction_msgs::Gestures& gestures);

protected:
  int skeleton_id_;
  std::list<tf::Transform> left_elbow_path_;
  std::list<tf::Transform> right_elbow_path_;
  double min_flapping_length_;
  double detect_rate_;
  ros::Time last_detected_time_;
};

}


#endif /* ELBOW_FLAPPING_SKELETON_GESTURE_DETECTOR_H_ */
