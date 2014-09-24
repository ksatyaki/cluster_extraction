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
#include <pcl/common/centroid.h>

#include <doro_msgs/ClusterArray.h>

#define DIST(A, B) sqrt ( ((A.x()-B.x())*(A.x()-B.x())) + ((A.y()-B.y())*(A.y()-B.y())) + ((A.z()-B.z())*(A.z()-B.z())) )

typedef pcl::PointXYZ PoinT;

namespace cluster_extraction
{

union d_in_f
{
	double value;
	int ind[2];
};

/**
 * The class for cluster extraction nodelet.
 * This exploits zero-copy passing of PointCloud data.
 * This nodelet is loaded into the /xtion_camera/xtion_camera_nodelet_manager.
 */
class ClusterExtraction: public nodelet::Nodelet
{
protected:

	/**
	 * A variable to hold the subscription state.
	 */
	bool subscribed_;

	/**
	 * A Listener for transforms.
	 */
	tf::TransformListener *tf_listener;

	/**
	 * Time stamp for the latest data.
	 */
	ros::Time stamp_;

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
	//ros::Publisher table_coeffs_pub;

	/**
	 * ROS Publisher that publishes the centroid and size of clusters.
	 */
	ros::Publisher clusters_pub;

	/**
	 * ROS Subscriber for the point cloud.
	 */
	ros::Subscriber cloud_sub;
	
	/** 
	 * ROS Publisher for the filtered cloud to be used by Moveit.
	 */
	 ros::Publisher filtered_cloud_pub;

	 /**
	  * Visualize the table.
	  */
	 ros::Publisher table_cloud_pub;

	 /**
	  * A templated point cloud type to hold the data.
	  */
	 pcl::PointCloud<PoinT>::Ptr pcl_data;

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
	std::vector <double> getClusterDimensions(const pcl::PointCloud<PoinT>::ConstPtr& input_cluster, tf::StampedTransform& base_link_to_openni_transform);

	/**
	 * Callback for the point cloud data.
	 */
	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& _cloud);

	/**
	 * The function where the point cloud is processed.
	 */
	void processCloud();

	/**
	 * Return the number of planes detected.
	 */
	int getPlanes(const pcl::PointCloud<PoinT>::ConstPtr& _cloud);

	/**
	 * Return the number of cylinders detected.
	 */
	int getCylinders(const pcl::PointCloud<PoinT>::ConstPtr& _cloud);

};
} // namespace cluster_extraction
#endif /* CLUSTER_EXTRACTION_H_ */
