/*
 * cluster_extraction.h
 *
 *  Created on: Apr 15, 2014
 *      Author: ace
 */

#ifndef CLUSTER_EXTRACTION_H_
#define CLUSTER_EXTRACTION_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <doro_msgs/Clusters.h>

namespace cluster_extraction
{

/**
 * The class for cluster extraction nodelet.
 * This exploits zero-copy passing of PointCloud data.
 * This nodelet is loaded into the /xtion_camera/xtion_camera_nodelet_manager.
 */
class ClusterExtraction: public nodelet::Nodelet
{
protected:

	/**
	 * A Listener for transforms.
	 */
	tf::TransformListener *tf_listener;

	/**
	 * ROS Publisher for publishing the points message.
	 */
	ros::Publisher centroid_pub;

	/**
	 * ROS Publisher for the table centroid.
	 */
	ros::Publisher table_position_pub;

	/**
	 * ROS Publisher for the model coefficients of the table.
	 */
	ros::Publisher table_coeffs_pub;

	/**
	 * ROS Publisher that publishes the centroid and size of clusters.
	 */
	ros::Publisher clusters_pub;

	/**
	 * ROS Subscriber for the point cloud.
	 */
	ros::Subscriber cloud_sub;
	
	/** 
	 * ROS Publisher for the filtered cloud to be used by Moveit
	 */
	 ros::Publisher filtered_cloud_pub;

public:
	ClusterExtraction();
	virtual ~ClusterExtraction();

	/**
	 * The overloaded onInit() method. This is called when the nodelet is loaded.
	 */
	void onInit();

	/**
	 * Callback for the point cloud data.
	 */
	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& _cloud);
};
} // namespace cluster_extraction
#endif /* CLUSTER_EXTRACTION_H_ */
