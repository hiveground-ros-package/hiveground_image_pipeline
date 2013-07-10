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

#ifndef KALMAN_FILTER3D_H_
#define KALMAN_FILTER3D_H_

#include <opencv2/opencv.hpp>

class KalmanFilter3d
{
public:
  enum
  {
    ERROR = -1,
    START = 0,
    TRACK,
    LOST,
    DIE
  };

  enum
  {
    MESUREMENT = 0,
    PREDICTED,
    ESTIMATED
  };


  KalmanFilter3d();
  ~KalmanFilter3d();

  void initialize(double dt,
                  const cv::Point3f& point,
                  double process_noise = 1e-4,
                  double measurement_noise = 1e-1,
                  double error_cov = 1e-1,
                  int update_before_track = 5,
                  int predict_before_die = 10);
  void predict(cv::Mat& result);
  void update(const cv::Point3f& measurement, cv::Mat& result);
  cv::Mat lastResult() { return last_result_; }
  int updateState();
  int getState() { return current_state_; }
  int id() { return id_; }

protected:
  cv::KalmanFilter filter_;
  static int id_count_;
  int id_;
  int predict_count_;
  int update_count_;
  int current_state_;
  int update_before_track_;
  int predict_before_die_;
  cv::Mat last_result_;
};


#endif /* KALMAN_FILTER3D_H_ */
