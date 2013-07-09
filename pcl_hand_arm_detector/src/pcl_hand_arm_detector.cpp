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
#include <boost/thread.hpp>
#include <tf/tf.h>



#include <pcl_hand_arm_detector/kalman_filter3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/pca.h>
#include <pcl/io/io.h>
#include <pcl/common/centroid.h>

#include <dynamic_reconfigure/server.h>
#include <pcl_hand_arm_detector/PclHandArmDetectorConfig.h>

using namespace visualization_msgs;

ros::Subscriber g_clustered_clouds_sub;
ros::Publisher g_marker_array_pub;
ros::Publisher g_cloud_pub;

static const int HAND_HISTORY_SIZE = 30;
std::vector<KalmanFilter3d> g_hand_trackers;
std::vector<std::list<geometry_msgs::Point> > g_hand_histories;


MarkerArray g_marker_array;
int g_marker_id = 0;
pcl_hand_arm_detector::PclHandArmDetectorConfig g_config;
boost::mutex mutex_config;


void callbackConfig(pcl_hand_arm_detector::PclHandArmDetectorConfig &config, uint32_t level)
{
  mutex_config.lock();
  g_config = config;
  mutex_config.unlock();


  //g_sac_distance_threshold = config.sac_dist_threshold;
  //g_ec_cluster_tolerance = config.ec_cluster_tolerance;
  //g_ec_min_cluster_size = config.ec_min_size;
  //g_ec_max_cluster_size = config.ec_max_size;
}

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

void pushSimpleMarker(double x, double y, double z,
                      double r, double g, double b,
                      double scale,
                      int& marker_id,
                      MarkerArray& marker_array,
                      const std::string& frame_id)
{
  Marker marker;
  marker.type = Marker::SPHERE;
  marker.lifetime = ros::Duration(0.1);
  marker.header.frame_id = frame_id;
  marker.ns = "simple_marker";
  marker.id = marker_id++;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker_array.markers.push_back(marker);
}


template <class T>
bool checkCloud(const sensor_msgs::PointCloud2& cloud_msg,
                typename pcl::PointCloud<T>::Ptr hand_cloud,
                //typename pcl::PointCloud<T>::Ptr finger_cloud,
                const std::string& frame_id,
                tf::Vector3& hand_position,
                tf::Vector3& arm_direction)
{  
  typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>);
  pcl::fromROSMsg(cloud_msg, *cloud);

  if((cloud->points.size() < g_config.min_cluster_size) ||
     (cloud->points.size() > g_config.max_cluster_size))
    return false;

  pcl::PCA<T> pca;
  pca.setInputCloud(cloud);
  Eigen::Vector4f mean = pca.getMean();

  if((mean.coeff(0) < g_config.min_x) || (mean.coeff(0) > g_config.max_x)) return false;
  if((mean.coeff(1) < g_config.min_y) || (mean.coeff(1) > g_config.max_y)) return false;
  if((mean.coeff(2) < g_config.min_z) || (mean.coeff(2) > g_config.max_z)) return false;

  Eigen::Vector3f eigen_value = pca.getEigenValues();
  double ratio = eigen_value.coeff(0) / eigen_value.coeff(1);

  if((ratio < g_config.min_eigen_value_ratio) || (ratio > g_config.max_eigen_value_ratio)) return false;

  T search_point;
  Eigen::Matrix3f ev = pca.getEigenVectors();
  Eigen::Vector3f main_axis(ev.coeff(0, 0), ev.coeff(1, 0), ev.coeff(2, 0));

  main_axis = (-main_axis.normalized() * 0.3) + Eigen::Vector3f(mean.coeff(0), mean.coeff(1), mean.coeff(2));
  search_point.x = main_axis.coeff(0);
  search_point.y = main_axis.coeff(1);
  search_point.z = main_axis.coeff(2);
  main_axis.normalize();

  //find hand
  pcl::KdTreeFLANN<T> kdtree;
  kdtree.setInputCloud(cloud);

  //find the closet point from the serach_point
  std::vector<int> point_indeices(1);
  std::vector<float> point_distances(1);
  if ( kdtree.nearestKSearch (search_point, 1, point_indeices, point_distances) > 0 )
  {
    //update search point
    search_point = cloud->points[point_indeices[0]];

    //show seach point
    if(g_marker_array_pub.getNumSubscribers() != 0)
    {
      pushSimpleMarker(search_point.x, search_point.y, search_point.z,
                       1.0, 0, 0,
                       0.02,
                       g_marker_id, g_marker_array, frame_id);
    }

    //hand
    point_indeices.clear();
    point_distances.clear();
    kdtree.radiusSearch(search_point, g_config.hand_lenght, point_indeices, point_distances);
    for (size_t i = 0; i < point_indeices.size (); ++i)
    {
      hand_cloud->points.push_back(cloud->points[point_indeices[i]]);
      hand_cloud->points.back().r = 255;
      hand_cloud->points.back().g = 0;
      hand_cloud->points.back().b = 0;
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*hand_cloud, centroid);

    if(g_marker_array_pub.getNumSubscribers() != 0)
    {
      pushSimpleMarker(centroid.coeff(0), centroid.coeff(1), centroid.coeff(2),
                       0.0, 1.0, 0,
                       0.02,
                       g_marker_id, g_marker_array, frame_id);
    }

    hand_position.setX(centroid.coeff(0));
    hand_position.setY(centroid.coeff(1));
    hand_position.setZ(centroid.coeff(2));
    arm_direction.setX(main_axis.coeff(0));
    arm_direction.setY(main_axis.coeff(1));
    arm_direction.setZ(main_axis.coeff(2));

#if 0
    //fingers
    search_point.x = centroid.coeff(0);
    search_point.y = centroid.coeff(1);
    search_point.z = centroid.coeff(2);
    std::vector<int> point_indeices_inner;
    std::vector<float> point_distances_inner;
    kdtree.radiusSearch(search_point, 0.07, point_indeices_inner, point_distances_inner);

    std::vector<int> point_indeices_outter;
    std::vector<float> point_distances_outter;
    kdtree.radiusSearch(search_point, 0.1, point_indeices_outter, point_distances_outter);

    //ROS_INFO("before %d %d", point_indeices_inner.size(), point_indeices_outter.size());

    std::vector<int>::iterator it;
    for(size_t i = 0; i < point_indeices_inner.size(); i++)
    {

      it =  std::find(point_indeices_outter.begin(), point_indeices_outter.end(), point_indeices_inner[i]);
      if(it != point_indeices_outter.end())
      {
        point_indeices_outter.erase(it);
      }
    }

    //ROS_INFO("after %d %d", point_indeices_inner.size(), point_indeices_outter.size());

    //ROS_DEBUG_THROTTLE(1.0, "found %lu", point_indeices.size ());
    for (size_t i = 0; i < point_indeices_outter.size (); ++i)
    {
      finger_cloud->points.push_back(cloud->points[point_indeices_outter[i]]);
      finger_cloud->points.back().r = 255;
      finger_cloud->points.back().g = 0;
      finger_cloud->points.back().b = 0;
    }
#endif

  }

  if(g_marker_array_pub.getNumSubscribers() != 0)
    pushEigenMarker<T>(pca, g_marker_id, g_marker_array, 0.1, frame_id);


  return true;
}

int closestHand(const tf::Vector3& point, const std::vector<tf::Vector3>& hand_positions)
{
  double dist, min_distant = 1e6;
  size_t index = 0;
  for (size_t i = 0; i < hand_positions.size(); i++)
  {
    tf::Vector3 point2(hand_positions[i].x(),
                       hand_positions[i].y(),
                       hand_positions[i].z());
    dist = point.distance(point2);
    if (dist < min_distant)
    {
      min_distant = dist;
      index = i;
    }
  }
  return index;
}


void callbackClusteredClouds(const clustered_clouds_msgs::ClusteredCloudsConstPtr& msg)
{
  g_marker_array.markers.clear();
  g_marker_id = 0;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<tf::Vector3> hand_positions, arm_directions;
  mutex_config.lock();
  for(size_t i = 0; i < msg->clouds.size(); i++)
  {
    bool cloud_with_rgb_data = false;
    std::string field_list = pcl::getFieldsList (msg->clouds[i]);    
    if(field_list.rfind("rgb") != std::string::npos)
    {
      cloud_with_rgb_data = true;
    }



    if(cloud_with_rgb_data)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      //pcl::PointCloud<pcl::PointXYZRGB>::Ptr finger_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      tf::Vector3 hand_position, arm_direction;
      bool found = checkCloud<pcl::PointXYZRGB>(msg->clouds[i],
                                                hand_cloud,
                                                //finger_cloud,
                                                msg->header.frame_id,
                                                hand_position,
                                                arm_direction);
      if(found)
      {
        hand_positions.push_back(hand_position);
        arm_directions.push_back(arm_direction);
        *cloud_out += *hand_cloud;
      }
    }
    //else
    //{
      //checkCloud<pcl::PointXYZ>(msg->clouds[i], msg->header.frame_id);
    //}
  }
  mutex_config.unlock();

  if(hand_positions.size() != g_hand_trackers.size())
  {
    g_hand_trackers.clear();
    g_hand_histories.resize(hand_positions.size());
    for (size_t i = 0; i < hand_positions.size(); i++)
    {
      KalmanFilter3d tracker;
      cv::Point3f point(hand_positions[i].x(),
                        hand_positions[i].y(),
                        hand_positions[i].z());
      tracker.initialize(1.0 / 30.0, point, 1e-2, 1e-1, 1e-1);
      g_hand_trackers.push_back(tracker);
    }
  }

  geometry_msgs::Point marker_point;
  cv::Point3f measurement;
  cv::Mat result;
  int state;
  for (size_t i = 0; i < g_hand_trackers.size(); i++)
  {
    if (i + 1 <= hand_positions.size())
    {
      g_hand_trackers[i].predict(result);
      //ROS_INFO_STREAM("[" << i << "] predicted: " << result);
      marker_point.x = result.at<float>(0);
      marker_point.y = result.at<float>(1);
      marker_point.z = result.at<float>(2);

      tf::Vector3 point1(result.at<float>(0), result.at<float>(1), result.at<float>(2));
      int index = closestHand(point1, hand_positions);
      state = g_hand_trackers[i].getState();

      switch (state)
      {
        case KalmanFilter3d::START:
        case KalmanFilter3d::TRACK:
        case KalmanFilter3d::LOST:
        {
          //ROS_INFO("%d TRACK", i);
          measurement.x = hand_positions[index].x();
          measurement.y = hand_positions[index].y();
          measurement.z = hand_positions[index].z();
          break;
        }
        case KalmanFilter3d::DIE:
          //ROS_INFO("%d DIE", i);
          cv::Point3f point(hand_positions[index].x(),
                            hand_positions[index].y(),
                            hand_positions[index].z());
          //ROS_INFO_STREAM("[" << i << "] point: " << point);
          g_hand_trackers[i].initialize(1.0 / 30.0, point, 1e-2, 1e-1, 1e-1);
          break;
      }

      if ((state == KalmanFilter3d::TRACK) || (state == KalmanFilter3d::START))
      {
        g_hand_trackers[i].update(measurement, result);

        marker_point.x = result.at<float>(0);
        marker_point.y = result.at<float>(1);
        marker_point.z = result.at<float>(2);
        g_hand_histories[i].push_back(marker_point);

        if (g_hand_histories[i].size() > HAND_HISTORY_SIZE)
        {
          g_hand_histories[i].pop_front();
        }
      }
    }
  }

  if(g_marker_array_pub.getNumSubscribers() != 0)
  {
    Marker marker;
    marker.type = Marker::LINE_STRIP;
    marker.lifetime = ros::Duration(0.1);
    marker.header.frame_id = msg->header.frame_id;
    marker.scale.x = 0.005;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.ns = "point_history";

    for (size_t i = 0; i < g_hand_histories.size(); i++)
    {
      marker.points.clear();
      if (g_hand_histories[i].size() != 0)
      {
        marker.id = g_marker_id++;
        std::list<geometry_msgs::Point>::iterator it;
        for (it = g_hand_histories[i].begin(); it != g_hand_histories[i].end(); it++)
        {
          marker.points.push_back(*it);
        }
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1.0;
        g_marker_array.markers.push_back(marker);
      }
    }
  }



  if(g_cloud_pub.getNumSubscribers() != 0)
  {
    sensor_msgs::PointCloud2 cloud_out_msg;
    pcl::toROSMsg(*cloud_out, cloud_out_msg);
    cloud_out_msg.header.stamp = ros::Time::now();
    cloud_out_msg.header.frame_id = msg->header.frame_id;
    g_cloud_pub.publish(cloud_out_msg);
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

  dynamic_reconfigure::Server<pcl_hand_arm_detector::PclHandArmDetectorConfig> server;
  dynamic_reconfigure::Server<pcl_hand_arm_detector::PclHandArmDetectorConfig>::CallbackType f;

  f = boost::bind(&callbackConfig, _1, _2);
  server.setCallback(f);

  g_clustered_clouds_sub = nh.subscribe("clustered_clouds", 1, callbackClusteredClouds);
  g_marker_array_pub = nhp.advertise<MarkerArray>("hand_arm_markers", 128);
  g_cloud_pub = nhp.advertise<sensor_msgs::PointCloud2>("cloud_output", 1);
  
  ros::spin();
  return 0;
}

