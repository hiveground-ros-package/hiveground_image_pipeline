/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
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
 *      * Neither the name of the Imai Laboratory, nor the name of its
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
#include <pcl_object_clustering/kalman_filter3d.h>


KalmanFilter3d::KalmanFilter3d()
  : predict_count_(0), update_count_(0),
    current_state_(START)
{

}

KalmanFilter3d::~KalmanFilter3d()
{

}

void KalmanFilter3d::initialize(double dt,
                                    const cv::Point3f& point,
                                    double process_noise,
                                    double measurement_noise,
                                    double error_cov)
{
  filter_ = cv::KalmanFilter(6, 3, 0);
  filter_.statePost.at<float>(0) = point.x;
  filter_.statePost.at<float>(1) = point.y;
  filter_.statePost.at<float>(2) = point.z;
  filter_.statePost.at<float>(3) = 0.0;
  filter_.statePost.at<float>(4) = 0.0;
  filter_.statePost.at<float>(5) = 0.0;
  filter_.transitionMatrix = (cv::Mat_<float>(6, 6) <<
      1, 0, 0, dt, 0, 0,
      0, 1, 0, 0, dt, 0,
      0, 0, 1, 0, 0, dt,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1);
  cv::setIdentity(filter_.measurementMatrix);
  cv::setIdentity(filter_.processNoiseCov, cv::Scalar::all(process_noise));
  cv::setIdentity(filter_.measurementNoiseCov, cv::Scalar::all(measurement_noise));
  cv::setIdentity(filter_.errorCovPost, cv::Scalar::all(error_cov));
  predict_count_ = 0;
  update_count_ = 0;
  current_state_ = START;
}

void KalmanFilter3d::predict(cv::Mat& result)
{
  result = filter_.predict();
  predict_count_++;
}

void KalmanFilter3d::update(const cv::Point3f& measurement, cv::Mat& result)
{
  cv::Mat_<float> z(3,1);
  z.setTo(cv::Scalar(0));
  z(0) = measurement.x;
  z(1) = measurement.y;
  z(2) = measurement.z;
  result = filter_.correct(z);
  update_count_++;
}

int KalmanFilter3d::updateState()
{
  switch(current_state_)
  {
    case START:
      if(predict_count_ > update_count_)
      {
        current_state_ = DIE;
      }
      else if(predict_count_ == update_count_)
      {
        if(update_count_ > 5)
        {
          current_state_ = TRACK;
        }
      }
      else
      {
        ROS_ERROR("Should not happen! call predict before update!");
      }
      break;
    case TRACK:
      if(predict_count_ > update_count_)
      {
        current_state_ = LOST;
      }
      else if(predict_count_ == update_count_)
      {
          current_state_ = TRACK;
      }
      else
      {
        ROS_ERROR("Should not happen! call predict before update!");
      }
      break;
    case LOST:
      if((predict_count_ - update_count_) > 30)
      {
        predict_count_ = 0;
        update_count_ = 0;
        current_state_ = DIE;
      }
      break;
    case DIE:
      if(update_count_ > 0)
      {
        current_state_ = START;
      }
      break;
  }
  return current_state_;
}




