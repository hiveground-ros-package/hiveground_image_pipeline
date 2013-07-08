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
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_object_clustering/pcl_object_clustering.h>
#include <pcl_object_clustering/PclObjectClusteringConfig.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <pcl/io/io.h>
#include <clustered_clouds_msgs/ClusteredClouds.h>


ros::Subscriber g_cloud_sub;
ros::Publisher g_cloud_pub;
ros::Publisher g_clustered_clouds_pub;

double g_sac_distance_threshold;
double g_ec_cluster_tolerance;
int g_ec_min_cluster_size;
int g_ec_max_cluster_size;
int g_marker_id = 0;

using namespace std;

void callbackConfig(pcl_object_clustering::PclObjectClusteringConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %f %f %d %d",
           config.sac_dist_threshold,
           config.ec_cluster_tolerance,
           config.ec_min_size,
           config.ec_max_size);
  g_sac_distance_threshold = config.sac_dist_threshold;
  g_ec_cluster_tolerance = config.ec_cluster_tolerance;
  g_ec_min_cluster_size = config.ec_min_size;
  g_ec_max_cluster_size = config.ec_max_size;
}

template <class T>
void sacSegmentation(typename pcl::PointCloud<T>::Ptr in,
                     typename pcl::PointCloud<T>::Ptr out_planar,
                     typename pcl::PointCloud<T>::Ptr out_objects)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<T> sac_segmentator;
  sac_segmentator.setOptimizeCoefficients(true);
  sac_segmentator.setModelType(pcl::SACMODEL_PLANE);
  sac_segmentator.setMethodType(pcl::SAC_RANSAC);
  sac_segmentator.setMaxIterations (1000);
  sac_segmentator.setDistanceThreshold(g_sac_distance_threshold);
  sac_segmentator.setInputCloud(in->makeShared());
  sac_segmentator.segment(*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    ROS_ERROR("Could not estimate a planar model for the given dataset.");
    return;
  }
  else
  {
    ROS_DEBUG("Indices size: %d", (int)inliers->indices.size ());
  }

  pcl::ExtractIndices<T> indices_extractor;
  indices_extractor.setNegative(false);
  indices_extractor.setInputCloud(in);
  indices_extractor.setIndices(inliers);
  indices_extractor.filter(*out_planar);

  indices_extractor.setNegative(true);
  indices_extractor.filter(*out_objects);
}

template <class T>
void objectSegmentation(typename pcl::PointCloud<T>::Ptr in,
                        std::vector<typename pcl::PointCloud<T>::Ptr>& out)
{
  typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>);
  tree->setInputCloud(in);
  std::vector<pcl::PointIndices> cluster_indices;


  pcl::EuclideanClusterExtraction<T> ec_extractor;
  ec_extractor.setClusterTolerance(g_ec_cluster_tolerance);
  ec_extractor.setMinClusterSize(g_ec_min_cluster_size);
  ec_extractor.setMaxClusterSize(g_ec_max_cluster_size);
  ec_extractor.setSearchMethod(tree);
  ec_extractor.setInputCloud(in);
  ec_extractor.extract(cluster_indices);

  ROS_DEBUG_THROTTLE(1.0, "total object %d", (int)cluster_indices.size());

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    typename pcl::PointCloud<T>::Ptr cloud_cluster(new pcl::PointCloud<T>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    {
      cloud_cluster->points.push_back(in->points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    out.push_back(cloud_cluster);
  }
}

template <class T>
void processCloud(const sensor_msgs::PointCloud2ConstPtr& msg, clustered_clouds_msgs::ClusteredClouds& msg_out)
{
  typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>);
  pcl::fromROSMsg(*msg, *cloud);
  typename pcl::PointCloud<T>::Ptr cloud_planar(new pcl::PointCloud<T>);
  typename pcl::PointCloud<T>::Ptr cloud_objects(new pcl::PointCloud<T>);
  sacSegmentation<T>(cloud, cloud_planar, cloud_objects);

  std::vector<typename pcl::PointCloud<T>::Ptr> clustered_clouds;
  objectSegmentation<T>(cloud_objects, clustered_clouds);

  msg_out.clouds.resize(clustered_clouds.size());
  for(size_t i = 0; i < clustered_clouds.size(); i++)
  {
    pcl::toROSMsg(*clustered_clouds[i], msg_out.clouds[i]);
  }
}


void callbackCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  //MarkerArray marker_array;
  g_marker_id = 0;

  bool cloud_with_rgb_data = false;
  std::string field_list = pcl::getFieldsList (*msg);
  ROS_DEBUG_STREAM_ONCE("Cloud type: " << field_list);
  if(field_list.rfind("rgb") != std::string::npos)
  {
    cloud_with_rgb_data = true;
  }

  clustered_clouds_msgs::ClusteredClouds msg_out;
  if(cloud_with_rgb_data)
  {
    processCloud<pcl::PointXYZRGB>(msg, msg_out);
  }
  else
  {
    processCloud<pcl::PointXYZ>(msg, msg_out);
  }

  if(g_clustered_clouds_pub.getNumSubscribers() != 0 && msg_out.clouds.size() != 0)
  {
    msg_out.header.stamp = ros::Time::now();
    msg_out.header.frame_id = msg->header.frame_id;
    g_clustered_clouds_pub.publish(msg_out);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_clustering");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  dynamic_reconfigure::Server<pcl_object_clustering::PclObjectClusteringConfig> server;
  dynamic_reconfigure::Server<pcl_object_clustering::PclObjectClusteringConfig>::CallbackType f;

  f = boost::bind(&callbackConfig, _1, _2);
  server.setCallback(f);

  g_cloud_sub = nh.subscribe("cloud_in", 1, callbackCloud);
  g_cloud_pub = nhp.advertise<sensor_msgs::PointCloud2>("cloud_output", 1);
  g_clustered_clouds_pub = nhp.advertise<clustered_clouds_msgs::ClusteredClouds>("clustered_clouds_output", 1);

  ros::spin();
  return 0;
}



