/*
 * cluster_extraction.h
 *
 *  Created on: Apr 15, 2014
 *      Author: ace
 */

#pragma once

#include <thread>

#include <nodelet/nodelet.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <cluster_extraction/ClusterArray.h>

#define DIST(A, B)                           \
  sqrt(((A.x() - B.x()) * (A.x() - B.x())) + \
       ((A.y() - B.y()) * (A.y() - B.y())) + \
       ((A.z() - B.z()) * (A.z() - B.z())))

typedef pcl::PointXYZRGB PoinT;

namespace cluster_extraction {

/**
 * The class for cluster extraction nodelet.
 * This exploits zero-copy passing of PointCloud data.
 * This nodelet is loaded into the /xtion_camera/xtion_camera_nodelet_manager.
 */
class ClusterExtraction : public nodelet::Nodelet {
 protected:
  /**
   * A variable to hold the subscription state.
   */
  bool subscribed_;

  /**
   * A nodehandle for this class.
   */
  ros::NodeHandle nh_;

  /**
   * A Listener for transforms.
   */
  std::shared_ptr<tf::TransformListener> tf_listener;

  /**
   * ROS Publisher that publishes the centroid and size of clusters.
   */
  ros::Publisher clusters_pub;

  /**
   * ROS Subscriber for the point cloud.
   */
  ros::Subscriber cloud_sub;

  /**
   * A templated point cloud type to hold the data.
   */
  sensor_msgs::PointCloud2ConstPtr pcl_data;

  /**
   * A thread id for the cluster_extraction thread.
   */
  std::thread c_e_thread_id_;

  /**
   * The thread where we do the polling.
   */
  void clusterExtractionThread();

 public:
  ClusterExtraction();
  virtual ~ClusterExtraction();

  /**
   * The overloaded onInit() method. This is called when the nodelet is loaded.
   */
  void onInit();

 protected:
  /**
   * Compute approximate dimentions of the objects seen.
   */
  std::vector<double> computeClusterDimensions(
      const pcl::PointCloud<PoinT>::ConstPtr& input_cluster,
      geometry_msgs::Point& a, geometry_msgs::Point& b);

  /**
   * Callback for the point cloud data.
   */
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& _cloud);

  /**
   * The function where the point cloud is processed.
   */
  void processCloud(float plane_tolerance);

  /**
   * Return the number of planes detected.
   */
  int getPlanes(const pcl::PointCloud<PoinT>::ConstPtr& _cloud);

  /**
   * Return the number of cylinders detected.
   */
  int getCylinders(const pcl::PointCloud<PoinT>::ConstPtr& _cloud);
};
}  // namespace cluster_extraction
