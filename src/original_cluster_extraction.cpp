/*
 * cluster_extraction.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: ace
 */

#include "cluster_extraction/original_cluster_extraction.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cluster_extraction::ClusterExtraction, nodelet::Nodelet);

namespace cluster_extraction
{

ClusterExtraction::ClusterExtraction()
{
	// TODO Auto-generated constructor stub

}

ClusterExtraction::~ClusterExtraction()
{

}

void ClusterExtraction::onInit()
{
	bool cluster_extraction_enable;
	ros::NodeHandle nh;

	bool hasparam = nh.getParam("/cluster_extraction_enable", cluster_extraction_enable);

	if(!hasparam || ( hasparam && !cluster_extraction_enable) )
	{
		NODELET_INFO("NOTE: The Parameter /cluster_extraction_enable was not set to true. Default value is 'false'. Cloud will not be processed.");
	}

	else NODELET_INFO("Cluster Extraction has started!");

	tf_listener = new tf::TransformListener(nh, ros::Duration(10));
	
	filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/manipulation_scene", 1);

	table_position_pub = nh.advertise<geometry_msgs::PointStamped> ("/table_position", 1);

	table_coeffs_pub = nh.advertise<pcl_msgs::ModelCoefficients> ("/table_coeffs", 1);

	clusters_pub = nh.advertise<doro_msgs::Clusters> ("/clusters", 1);

	cloud_sub = nh.subscribe("/xtion_camera/depth/points", 10 , &ClusterExtraction::cloudCallback, this);

}

void ClusterExtraction::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& _cloud)
{
	bool cluster_extraction_enable = false, plane_extraction_enable = false;
	bool has_cluster_param = ros::param::get("/cluster_extraction_enable", cluster_extraction_enable);
	bool has_plane_param = ros::param::get("/plane_extraction_enable", plane_extraction_enable);

	// Convert the ROS message to a pcl point cloud (templated) type.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg (*_cloud, *cloud);

	// Filter.
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

	vg.setInputCloud (cloud);
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*cloud_filtered);

	// Findout the points that are more than 1.25 m away.
	pcl::PointIndices::Ptr out_of_range_points (new pcl::PointIndices);
	int i = 0;
	BOOST_FOREACH(const pcl::PointXYZ& pt, cloud_filtered->points)
	{
	  if(sqrt( (pt.x*pt.x) + (pt.y*pt.y) + (pt.z*pt.z) ) > 1.25)
		  out_of_range_points->indices.push_back(i);
	  i++;
	}

	pcl::ExtractIndices<pcl::PointXYZ> extract;

	// Perform the extraction of these points (indices).
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(out_of_range_points);
	extract.setNegative (true);
	extract.filter (*cloud_f);
	*cloud_filtered = *cloud_f;

	filtered_cloud_pub.publish(cloud_f);


	if( (has_cluster_param && cluster_extraction_enable) || (has_plane_param && plane_extraction_enable) )
	{
	// Prepare plane segmentation.
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.02);

	// Remove the planes.
	i = 0;
	int max_planes = 3;
	int nr_points = cloud_filtered->points.size();

	//while (max_planes-- && cloud_filtered->points.size () > 0.3 * nr_points)
	//{
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);

		if(inliers->indices.size () < 200) return;
			//break;

		// Extract the planar inliers from the input cloud
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_plane);
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud_filtered = *cloud_f;
	//}

    //////////////////////////////////////////////////////////////////////
    /**
     * COMPUTE THE CENTROID OF THE PLANE AND PUBLISH IT.
     */
    //////////////////////////////////////////////////////////////////////
    geometry_msgs::PointStamped plane_centroid;
    plane_centroid.point.x = 0.0;
    plane_centroid.point.y = 0.0;
    plane_centroid.point.z = 0.0;
    plane_centroid.header.frame_id = _cloud->header.frame_id;
    plane_centroid.header.stamp = ros::Time::now();

    geometry_msgs::PointStamped plane_normal;
    plane_normal.point.x = coefficients->values[0];
    plane_normal.point.y = coefficients->values[1];
    plane_normal.point.z = coefficients->values[2];
    plane_normal.header.frame_id = plane_centroid.header.frame_id;
    plane_normal.header.stamp = plane_centroid.header.stamp;


    // Centroid computation.
    BOOST_FOREACH(pcl::PointXYZ p, cloud_plane->points)
    {
    	if(isnan(p.x) || isnan(p.y) || isnan(p.z))
    		continue;
    	plane_centroid.point.x += p.x;
    	plane_centroid.point.y += p.y;
    	plane_centroid.point.z += p.z;
    }

    plane_centroid.point.x = plane_centroid.point.x/ (double) ((cloud_plane->points).size());
    plane_centroid.point.y = plane_centroid.point.y/ (double) ((cloud_plane->points).size());
    plane_centroid.point.z = plane_centroid.point.z/ (double) ((cloud_plane->points).size());

    geometry_msgs::PointStamped _plane_centroid, _plane_normal;
    try
    {
    	tf_listener->waitForTransform("base_link", plane_centroid.header.frame_id, plane_centroid.header.stamp, ros::Duration(1));
    	tf_listener->transformPoint("base_link", plane_centroid, _plane_centroid);
    	tf_listener->transformPoint("base_link", plane_normal, _plane_normal);
    }
    catch(tf::TransformException& ex)
    {
    	ROS_INFO("COCKED UP POINT INFO! Why: %s", ex.what());
    }

    //std::cout<<"FRESH COEFF: "<<*coefficients<<std::endl;
    //std::cout<<"SAME AGAIN: "<<plane_normal<<std::endl;
    //std::cout<<"AFTER TRANSFORMATION: "<<_plane_normal<<std::endl;

    double d = (_plane_normal.point.x*_plane_centroid.point.x)
    		+ (_plane_normal.point.y*_plane_centroid.point.y)
    		+ (_plane_normal.point.z*_plane_centroid.point.z);

    //ROS_INFO(" D is %f ", -1.00 * d);
    d *= -1.00;

    // Updated equation of the plane.
    coefficients->values[0] = _plane_normal.point.x;
    coefficients->values[1] = _plane_normal.point.y;
    coefficients->values[2] = _plane_normal.point.z;
    coefficients->values[3] = d;
    coefficients->header.frame_id = _plane_normal.header.frame_id;

    //std::cout<<"FINAL COEFF: "<<*coefficients<<std::endl;

    if((has_plane_param) && (plane_extraction_enable))
    {
    // Publish the centroid.
    table_position_pub.publish(_plane_centroid);

    pcl_msgs::ModelCoefficients table_coeffs;
    pcl_conversions::fromPCL(*coefficients, table_coeffs);
    // Publish the plane coeffs.
    table_coeffs_pub.publish(table_coeffs);
    }

    //////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////

    if(has_cluster_param && cluster_extraction_enable)
    {
	// Cluster Extraction
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);

	int j = 0;
	doro_msgs::Clusters clusters;

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*

		cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;
	    cloud_cluster->header.frame_id = _cloud->header.frame_id;

	    geometry_msgs::PointStamped cluster_centroid;
	    cluster_centroid.point.x = 0.0;
	    cluster_centroid.point.y = 0.0;
	    cluster_centroid.point.z = 0.0;
	    cluster_centroid.header.frame_id = cloud_cluster->header.frame_id;
	    cluster_centroid.header.stamp = ros::Time::now();

	    BOOST_FOREACH(pcl::PointXYZ p, cloud_cluster->points)
	    {
	    	cluster_centroid.point.x += p.x;
	    	cluster_centroid.point.y += p.y;
	    	cluster_centroid.point.z += p.z;
	    }

	    cluster_centroid.point.x = cluster_centroid.point.x/(cloud_cluster->points).size();
	    cluster_centroid.point.y = cluster_centroid.point.y/(cloud_cluster->points).size();
	    cluster_centroid.point.z = cluster_centroid.point.z/(cloud_cluster->points).size();
	    //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

	    geometry_msgs::PointStamped _cluster_centroid;
	    try
	    {
	       	tf_listener->waitForTransform("base_link", cluster_centroid.header.frame_id, cluster_centroid.header.stamp, ros::Duration(1));
	       	tf_listener->transformPoint("base_link", cluster_centroid, _cluster_centroid);
	    }
	    catch(tf::TransformException& ex)
	    {
	      	ROS_INFO("COCKED UP POINT INFO! Why: %s", ex.what());
	    }

	    clusters.cluster_centroids.push_back (_cluster_centroid);
	    clusters.cluster_sizes.push_back (cloud_cluster->width);

	}
	clusters_pub.publish(clusters);

	} // innter if
} // outer if

}

} // namespace cluster_extraction
