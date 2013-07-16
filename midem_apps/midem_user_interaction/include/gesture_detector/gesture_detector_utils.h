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

#ifndef GESTURE_DETECTOR_UTILS_H
#define GESTURE_DETECTOR_UTILS_H

#include <tf/tf.h>
#include <gesture_detector/gesture_detector_defines.h>
namespace hg_gesture_detector
{
  inline int checkDirection(const tf::Vector3& vec_to_point)
  {
    tf::Vector3 main_axes[3];
    main_axes[0] = tf::Vector3(1, 0, 0); //X
    main_axes[1] = tf::Vector3(0, 1, 0); //Y
    main_axes[2] = tf::Vector3(0, 0, 1); //Z

    double min_error = 1e6;
    int min_error_index = -1;
    double dot_products[3];
    for (int i = 0; i < 3; i++)
    {
      dot_products[i] = vec_to_point.dot(main_axes[i]);
      if ((1 - fabs(dot_products[i])) < min_error)
      {
        min_error = 1 - fabs(dot_products[i]);
        min_error_index = i;
      }
    }

    switch (min_error_index)
    {
      case 0:
        if (dot_products[min_error_index] > 0)
        {
          ROS_DEBUG("X+");
          return FORWARD;
        }
        else
        {
          ROS_DEBUG("X-");
          return BACKWARD;
        }
        break;
      case 1:
        if (dot_products[min_error_index] > 0)
        {
          ROS_DEBUG("Y+");
          return RIGHT;
        }
        else
        {
          ROS_DEBUG("Y-");
          return LEFT;
        }
        break;
      case 2:
        if (dot_products[min_error_index] > 0)
        {
          ROS_DEBUG("Z+");
          return UP;
        }
        else
        {
          ROS_DEBUG("Z-");
          return DOWN;
        }
        break;
      default:
        ROS_ERROR("Something wrong!");
        return UNKNOWN;
        break;
    }
    return UNKNOWN;
  }

  inline void rubberBandStateTransition(int& state,
                                             tf::Vector3& pivot_position,
                                             const tf::Vector3& last_position,
                                             ros::Time& start_activating_time,
                                             double activating_time,
                                             double r_activating,
                                             double r_activated,
                                             double r_leaving,
                                             double r_die)
  {
    tf::Vector3 vec_to_center = last_position - pivot_position;
    double distance = vec_to_center.length();
    switch(state)
    {
      case IDEL:
        pivot_position = last_position;
        state = ACTIVATING;
        start_activating_time = ros::Time::now();
        break;
      case ACTIVATING:
        if(distance < r_activating)
        {
          if((ros::Time::now() - start_activating_time).toSec() > activating_time)
          {
            state = ACTIVATED;
          }
        }
        else
        {
          state = IDEL;
        }
        break;
      case ACTIVATED:
        if(distance >= r_activated)
        {
          state = MOVING;
        }
        break;
      case MOVING:
        if(distance < r_activated)
        {
          state = ACTIVATED;
        }
        else if(distance >= r_leaving)
        {
          state = LEAVING;
        }
        break;
      case LEAVING:
        if(distance >= r_die)
        {
          state = IDEL;
        }
        else if(distance < r_leaving)
        {
          state = MOVING;
        }
        break;
    }
  }
}


#endif
