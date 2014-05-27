/*
 * cluster_extraction.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: ace
 */

#include "cluster_extraction/cluster_extraction.h"

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

	table_cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("/plane_cloud", 1);

	//table_coeffs_pub = nh.advertise<pcl_msgs::ModelCoefficients> ("/table_coeffs", 1);

	clusters_pub = nh.advertise<doro_msgs::Clusters> ("/clusters", 1);

	cloud_sub = nh.subscribe("/xtion_camera/depth/points", 10 , &ClusterExtraction::cloudCallback, this);

	while(ros::ok())
	{
		ros::spinOnce();
		processCloud();
	}

}

void ClusterExtraction::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& _cloud)
{
	bool cluster_extraction_enable = false, plane_extraction_enable = false;
	bool has_cluster_param = ros::param::get("/cluster_extraction_enable", cluster_extraction_enable);
	bool has_plane_param = ros::param::get("/plane_extraction_enable", plane_extraction_enable);

	if( (has_cluster_param && cluster_extraction_enable) || (has_plane_param && plane_extraction_enable) )
	{
		// Convert the ROS message to a pcl point cloud (templated) type.
		pcl_data = pcl::PointCloud<PoinT>::Ptr(new pcl::PointCloud<PoinT>);
		pcl::fromROSMsg (*_cloud, *pcl_data);
	}

	else
		if(pcl_data)
			pcl_data.reset();
}

void ClusterExtraction::processCloud()
{
	if(!pcl_data)
	{
		ROS_INFO("No data. Either the parameters are not set. Or there is a problem.");
		sleep(1);
		return;
	}

	bool cluster_extraction_enable = false, plane_extraction_enable = false;
	bool has_cluster_param = ros::param::get("/cluster_extraction_enable", cluster_extraction_enable);
	bool has_plane_param = ros::param::get("/plane_extraction_enable", plane_extraction_enable);

	// Make a copy of the class' data and then use it. Because a callback may occur as we are processing.

	pcl::PointCloud<PoinT>::Ptr _cloud (new pcl::PointCloud<PoinT>);
	*_cloud = *pcl_data;

	ros::Time stamp = ros::Time::now();

	pcl::VoxelGrid<PoinT> vg;
	pcl::PointCloud<PoinT>::Ptr cloud_filtered (new pcl::PointCloud<PoinT>);
	vg.setInputCloud (_cloud);
	vg.setLeafSize (0.01f, 0.01f, 0.01f);
	vg.filter (*cloud_filtered);

	*_cloud = *cloud_filtered;

	// Findout the points that are more than 1.25 m away.
	pcl::PointIndices::Ptr out_of_range_points (new pcl::PointIndices);
	int i = 0;
	BOOST_FOREACH(const PoinT& pt, _cloud->points)
	{
	  if(sqrt( (pt.x*pt.x) + (pt.y*pt.y) + (pt.z*pt.z) ) > 1.25)
		  out_of_range_points->indices.push_back(i);

	  //else if(isnan (pt.x) || isnan (pt.y) || isnan (pt.z) ||
		//	   isinf (pt.x) || isinf (pt.y) || isinf (pt.z) )
		//  out_of_range_points->indices.push_back(i);

	  i++;
	}

	pcl::ExtractIndices<PoinT> extract;
	pcl::PointCloud<PoinT>::Ptr cloud (new pcl::PointCloud<PoinT>);

	// Perform the extraction of these points (indices).
	extract.setInputCloud(_cloud);
	extract.setIndices(out_of_range_points);
	extract.setNegative (true);
	extract.filter (*cloud);

	filtered_cloud_pub.publish(cloud);

	// Prepare plane segmentation.
	pcl::SACSegmentation<PoinT> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

	pcl::PointCloud<PoinT>::Ptr cloud_plane; // This door is opened elsewhere.

	pcl::PointCloud<PoinT>::Ptr cloud_f (new pcl::PointCloud<PoinT> ());

	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.02);

	// Remove the planes.
	i = 0;

	int nr_points =  (int) cloud->points.size();

	while (cloud->points.size () > 0.4 * nr_points)
	{
		seg.setInputCloud (cloud);
		seg.segment (*inliers, *coefficients);

		if (inliers->indices.size () == 0)
		{
		  //std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
		  break;
		}

		// Extract the planar inliers from the input cloud
		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*cloud_f);

		// Is this a parallel to ground plane? If yes, save it.
		geometry_msgs::PointStamped plane_normal;
		plane_normal.point.x = coefficients->values[0];
		plane_normal.point.y = coefficients->values[1];
		plane_normal.point.z = coefficients->values[2];
		plane_normal.header.frame_id = cloud->header.frame_id;
		plane_normal.header.stamp = stamp;

		geometry_msgs::PointStamped _plane_normal; // In the base link frame.

		try
		{
			tf_listener->transformPoint("base_link", plane_normal, _plane_normal);
		}
		catch(tf::TransformException& ex)
		{
		  	ROS_INFO("COCKED UP POINT INFO! Why: %s", ex.what());
		}


		// What's the angle between this vector and the actual z axis? cos_inverse ( j component )...
		tf::Vector3 normal (_plane_normal.point.x, _plane_normal.point.y, _plane_normal.point.z);
		normal = normal.normalized();

		if(acos (normal.z()) < 0.1)
		{
			cloud_plane = pcl::PointCloud<PoinT>::Ptr(new pcl::PointCloud<PoinT>);
			*cloud_plane = *cloud_f;
		}

		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud = *cloud_f;
	}

	if(!cloud_plane)
	{
		ROS_INFO("No table or table-like object could be seen. Can't extract...");
		sleep(1);
		return;
	}

	table_cloud_pub.publish(cloud_plane);

    //////////////////////////////////////////////////////////////////////
    /**
     * COMPUTE THE CENTROID OF THE PLANE AND PUBLISH IT.
     */
    //////////////////////////////////////////////////////////////////////
    geometry_msgs::PointStamped plane_centroid;
    plane_centroid.point.x = 0.0;
    plane_centroid.point.y = 0.0;
    plane_centroid.point.z = 0.0;
    plane_centroid.header.frame_id = cloud->header.frame_id;
    plane_centroid.header.stamp = stamp;

/*
    // Centroid computation.
    BOOST_FOREACH(PoinT p, cloud_plane->points)
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

    */
    Eigen::Vector4f plane_cen;

    pcl::compute3DCentroid(*cloud_plane, plane_cen);

    plane_centroid.point.x = plane_cen[0];
    plane_centroid.point.y = plane_cen[1];
    plane_centroid.point.z = plane_cen[2];

    std::cout<< plane_cen;

    geometry_msgs::PointStamped _plane_centroid, _plane_normal;
    try
    {
    	tf_listener->waitForTransform("base_link", plane_centroid.header.frame_id, plane_centroid.header.stamp, ros::Duration(1));
    	tf_listener->transformPoint("base_link", plane_centroid, _plane_centroid);
    }
    catch(tf::TransformException& ex)
    {
    	ROS_INFO("COCKED UP POINT INFO! Why: %s", ex.what());
    }


    if((has_plane_param) && (plane_extraction_enable))
    {
    	// Publish the centroid.
    	table_position_pub.publish(_plane_centroid);
    }

    ////////////////////////////////////////////////////////////////////
    //////////////////////////// NAN FILTER ////////////////////////////
    ////////////////////////////////////////////////////////////////////

    pcl::PointIndices::Ptr nan_points (new pcl::PointIndices);
    i = 0;
    BOOST_FOREACH(const PoinT& pt, cloud->points)
    {

      if(isnan (pt.x) || isnan (pt.y) || isnan (pt.z) ||
    	  isinf (pt.x) || isinf (pt.y) || isinf (pt.z) )
    	  nan_points->indices.push_back(i);
   	  i++;
   	}

   	// Perform the extraction of these points (indices).
   	extract.setInputCloud(cloud);
   	extract.setIndices(nan_points);
   	extract.setNegative (true);
   	extract.filter (*cloud);

    //////////////////////////////////////////////////////////////////////

    if(has_cluster_param && cluster_extraction_enable)
    {
	// Cluster Extraction
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PoinT>::Ptr tree (new pcl::search::KdTree<PoinT>);
	tree->setInputCloud (cloud);


	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PoinT> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (50);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);

	ROS_INFO("GOOD TILL HERE!");
	ec.extract (cluster_indices);

	ROS_INFO("GOOD TILL HERE!");

	int j = 0;
	doro_msgs::Clusters clusters;

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<PoinT>::Ptr cloud_cluster (new pcl::PointCloud<PoinT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud->points[*pit]); //*

		cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;
	    cloud_cluster->header.frame_id = cloud->header.frame_id;

	    geometry_msgs::PointStamped cluster_centroid;
	    cluster_centroid.point.x = 0.0;
	    cluster_centroid.point.y = 0.0;
	    cluster_centroid.point.z = 0.0;
	    cluster_centroid.header.frame_id = cloud_cluster->header.frame_id;
	    cluster_centroid.header.stamp = stamp;

	    Eigen::Vector4f cluster_cen;

	    pcl::compute3DCentroid(*cloud_cluster, cluster_cen);

	    cluster_centroid.point.x = cluster_cen[0];
	    cluster_centroid.point.y = cluster_cen[1];
	    cluster_centroid.point.z = cluster_cen[2];

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

	    if(DIST(cluster_centroid.point,plane_centroid.point) < 0.3)
	    {
	    	clusters.cluster_centroids.push_back (_cluster_centroid);
	    	clusters.cluster_sizes.push_back (cloud_cluster->width);

	    	printf("**************************\n");
	    	printf("CLUSTER: %d\n",j);
	    	printf("Cluster has %d cylinder(s).\n", getCylinders(cloud_cluster) );
	    	printf("Cluster has %d planes(s).\n", getPlanes(cloud_cluster) );
	    	printf("**************************\n");
	    	j++;
	    }

	}
	clusters_pub.publish(clusters);
	}

    if(pcl_data)
    	pcl_data.reset();

}

int
ClusterExtraction::getCylinders (const pcl::PointCloud<PoinT>::ConstPtr& _cloud)
{
	pcl::NormalEstimation<PoinT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PoinT, pcl::Normal> seg;
	pcl::search::KdTree<PoinT>::Ptr tree (new pcl::search::KdTree<PoinT> ());

	pcl::PointCloud<pcl::Normal>::Ptr cloud_norm (new pcl::PointCloud<pcl::Normal>);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

	ne.setSearchMethod (tree);
	ne.setInputCloud (_cloud);
	ne.setKSearch (50);
	ne.compute (*cloud_norm);
	//pcl::PointCloud<PoinT>::Ptr cloud_plane; // This door is opened elsewhere.

	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_CYLINDER);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setRadiusLimits (0.01, 0.04);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.03);
	seg.setNormalDistanceWeight (0.02);


	pcl::ExtractIndices<PoinT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_norm;
	pcl::PointCloud<PoinT>::Ptr cloud (new pcl::PointCloud<PoinT>);
	*cloud = *_cloud;

	pcl::PointCloud<PoinT>::Ptr cloud_f (new pcl::PointCloud<PoinT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_norm_f (new pcl::PointCloud<pcl::Normal>);

	int nr_points = (int) cloud->points.size();

	int i;

	for(i = 0;cloud->points.size () > 0.4 * nr_points; i++)
	{
		seg.setInputNormals (cloud_norm);
		seg.setInputCloud (cloud);
		seg.segment (*inliers, *coefficients);

		if (inliers->indices.size () == 0)
		{
			//std::cout << "Could not estimate a model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (true);
		extract.filter (*cloud_f);

		//ROS_INFO("%d points -> pass %d\n", cloud->points.size(), i);
		*cloud = *cloud_f;
		//ROS_INFO("%d points -> pass %d after filters...\n\n", cloud->points.size(), i);

		// Same for normals
		extract_norm.setInputCloud (cloud_norm);
		extract_norm.setIndices (inliers);
		extract_norm.setNegative (true);
		extract_norm.filter (*cloud_norm_f);

		*cloud_norm = *cloud_norm_f;

		if (inliers->indices.size () < 80)
		{
			i--;
			//printf("SACMODELS: %f\n", coefficients->values[6]);
		}
	}

	return i;

}

int
ClusterExtraction::getPlanes (const pcl::PointCloud<PoinT>::ConstPtr& _cloud)
{
	pcl::SACSegmentation<PoinT> seg;
	pcl::search::KdTree<PoinT>::Ptr tree (new pcl::search::KdTree<PoinT> ());

	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	pcl::ExtractIndices<PoinT> extract;
	pcl::PointCloud<PoinT>::Ptr cloud (new pcl::PointCloud<PoinT>);
	*cloud = *_cloud;

	pcl::PointCloud<PoinT>::Ptr cloud_f (new pcl::PointCloud<PoinT>);

	int nr_points = (int) cloud->points.size();

	int i;

	for(i = 0;cloud->points.size () > 0.4 * nr_points; i++)
	{
		seg.setInputCloud (cloud);
		seg.segment (*inliers, *coefficients);

		if (inliers->indices.size () == 0)
		{
			//std::cout << "Could not estimate a model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		extract.setInputCloud (cloud);
		extract.setIndices (inliers);
		extract.setNegative (true);
		extract.filter (*cloud_f);

		//ROS_INFO("%d points -> pass %d\n", cloud->points.size(), i);
		*cloud = *cloud_f;
		//ROS_INFO("%d points -> pass %d after filters...\n\n", cloud->points.size(), i);
	}

	return i;
}

} // namespace cluster_extraction
