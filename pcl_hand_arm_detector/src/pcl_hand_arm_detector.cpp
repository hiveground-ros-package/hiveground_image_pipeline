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
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <clustered_clouds_msgs/ClusteredClouds.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <pcl_hand_arm_detector/kalman_filter3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>
#include <pcl/io/io.h>

using namespace visualization_msgs;

ros::Subscriber g_clustered_clouds_sub;
ros::Publisher g_marker_array_pub;
std::vector<KalmanFilter3d> g_hand_trackers;
MarkerArray g_marker_array;
int g_marker_id = 0;


template <class T>
void pushEigenMarker(pcl::PCA<T>& pca,
                     int& marker_id,
                     MarkerArray& marker_array,
                     double scale,
                     const std::string& frame_id)
{
  Marker marker;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = frame_id;
  marker.ns = "cluster_eigen";
  marker.type = Marker::ARROW;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.pose.position.x = pca.getMean().coeff(0);
  marker.pose.position.y = pca.getMean().coeff(1);
  marker.pose.position.z = pca.getMean().coeff(2);

  Eigen::Quaternionf qx, qy, qz;
  Eigen::Matrix3f ev = pca.getEigenVectors();
  Eigen::Vector3f axis_x(ev.coeff(0, 0), ev.coeff(1, 0), ev.coeff(2, 0));
  Eigen::Vector3f axis_y(ev.coeff(0, 1), ev.coeff(1, 1), ev.coeff(2, 1));
  Eigen::Vector3f axis_z(ev.coeff(0, 2), ev.coeff(1, 2), ev.coeff(2, 2));
  qx.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_x);
  qy.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_y);
  qz.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_z);

  marker.id = marker_id++;
  marker.scale.x = pca.getEigenValues().coeff(0) * scale;
  marker.pose.orientation.x = qx.x();
  marker.pose.orientation.y = qx.y();
  marker.pose.orientation.z = qx.z();
  marker.pose.orientation.w = qx.w();
  marker.color.b = 0.0;
  marker.color.g = 0.0;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker_array.markers.push_back(marker);

  marker.id = marker_id++;
  marker.scale.x = pca.getEigenValues().coeff(1) * scale;
  marker.pose.orientation.x = qy.x();
  marker.pose.orientation.y = qy.y();
  marker.pose.orientation.z = qy.z();
  marker.pose.orientation.w = qy.w();
  marker.color.b = 0.0;
  marker.color.g = 1.0;
  marker.color.r = 0.0;
  marker_array.markers.push_back(marker);

  marker.id = marker_id++;
  marker.scale.x = pca.getEigenValues().coeff(2) * scale;
  marker.pose.orientation.x = qz.x();
  marker.pose.orientation.y = qz.y();
  marker.pose.orientation.z = qz.z();
  marker.pose.orientation.w = qz.w();
  marker.color.b = 1.0;
  marker.color.g = 0.0;
  marker.color.r = 0.0;
  marker_array.markers.push_back(marker);
}

template <class T>
void checkCloud(const sensor_msgs::PointCloud2& cloud_msg, const std::string& frame_id)
{
  typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>);
  pcl::PCA<T> pca;
  pcl::fromROSMsg(cloud_msg, *cloud);
  pca.setInputCloud(cloud);
  Eigen::Vector4f mean = pca.getMean();

  pushEigenMarker<T>(pca, g_marker_id, g_marker_array, 0.1, frame_id);
}


void callbackClusteredClouds(const clustered_clouds_msgs::ClusteredCloudsConstPtr& msg)
{
  g_marker_array.markers.clear();
  g_marker_id = 0;

  for(size_t i = 0; i < msg->clouds.size(); i++)
  {
    bool cloud_with_rgb_data = false;
    std::string field_list = pcl::getFieldsList (msg->clouds[i]);
    ROS_INFO_STREAM_ONCE("Cloud type: " << field_list);
    if(field_list.rfind("rgb") != std::string::npos)
    {
      cloud_with_rgb_data = true;
    }

    if(cloud_with_rgb_data)
    {
      checkCloud<pcl::PointXYZRGB>(msg->clouds[i], msg->header.frame_id);
    }
    else
    {
      checkCloud<pcl::PointXYZ>(msg->clouds[i], msg->header.frame_id);
    }
  }

  if((g_marker_array_pub.getNumSubscribers() != 0) && (!g_marker_array.markers.empty()))
  {
    g_marker_array_pub.publish(g_marker_array);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_hand_arm_detector");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  g_clustered_clouds_sub = nh.subscribe("clustered_clouds", 1, callbackClusteredClouds);
  g_marker_array_pub = nhp.advertise<MarkerArray>("hand_arm_markers", 128);
  
  ros::spin();
  return 0;
}

