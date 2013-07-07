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
#include <clustered_object_msgs/ClusteredObjects.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>

using namespace visualization_msgs;

ros::Subscriber g_clustered_objects_sub;
ros::Publisher g_marker_array_pub;

typedef pcl::PointXYZ CLOUD_TYPE;

void pushEigenMarker(const std::vector<double>& mean, const std::vector<double>& eigen_values, const std::vector<double>& eigen_vectors, int& marker_id,
                     MarkerArray& marker_array, double scale, const std::string& frame_id)
{
  Marker marker;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = frame_id;
  marker.ns = "cluster_eigen";
  marker.type = Marker::ARROW;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.pose.position.x = mean[0];
  marker.pose.position.y = mean[1];
  marker.pose.position.z = mean[2];

  Eigen::Quaternionf qx, qy, qz;
  Eigen::Vector3f axis_x(eigen_vectors[0], eigen_vectors[3], eigen_vectors[6]);
  Eigen::Vector3f axis_y(eigen_vectors[1], eigen_vectors[4], eigen_vectors[7]);
  Eigen::Vector3f axis_z(eigen_vectors[2], eigen_vectors[5], eigen_vectors[8]);
  qx.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_x);
  qy.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_y);
  qz.setFromTwoVectors(Eigen::Vector3f(1, 0, 0), axis_z);

  marker.id = marker_id++;
  marker.scale.x = eigen_values[0] * scale;
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
  marker.scale.x = eigen_values[1] * 0.1;
  marker.pose.orientation.x = qy.x();
  marker.pose.orientation.y = qy.y();
  marker.pose.orientation.z = qy.z();
  marker.pose.orientation.w = qy.w();
  marker.color.b = 0.0;
  marker.color.g = 1.0;
  marker.color.r = 0.0;
  marker_array.markers.push_back(marker);

  marker.id = marker_id++;
  marker.scale.x = eigen_values[2] * 0.1;
  marker.pose.orientation.x = qz.x();
  marker.pose.orientation.y = qz.y();
  marker.pose.orientation.z = qz.z();
  marker.pose.orientation.w = qz.w();
  marker.color.b = 1.0;
  marker.color.g = 0.0;
  marker.color.r = 0.0;
  marker_array.markers.push_back(marker);
}

void callbackClusteredObject(const clustered_object_msgs::ClusteredObjectsPtr& msg)
{
  MarkerArray marker_array;
  int marker_id = 0;

  for(size_t i = 0; i < msg->objects.size(); i++)
  {      
    if(msg->objects[i].eigen_values[0] < (8*msg->objects[i].eigen_values[1])) continue;

    ROS_INFO("elongated object %lu %f %f %f %f", i,
             msg->objects[i].mean[0], msg->objects[i].mean[1],
             msg->objects[i].mean[2], msg->objects[i].mean[3]);

    pushEigenMarker(msg->objects[i].mean, msg->objects[i].eigen_values, msg->objects[i].eigen_vectors, marker_id, marker_array, 0.1, msg->header.frame_id);

    pcl::PointCloud<CLOUD_TYPE>::Ptr cloud(new pcl::PointCloud<CLOUD_TYPE>);
    pcl::fromROSMsg(msg->objects[i].cloud, *cloud);
  }

  if((g_marker_array_pub.getNumSubscribers() != 0) && (!marker_array.markers.empty()))
  {
    g_marker_array_pub.publish(marker_array);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_hand_arm_detector");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  g_clustered_objects_sub = nh.subscribe("clustered_objects", 1, callbackClusteredObject);
  g_marker_array_pub = nhp.advertise<MarkerArray>("hand_arm_markers", 128);
  
  ros::spin();
  return 0;
}

