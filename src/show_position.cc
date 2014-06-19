#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <signal.h>
#include <cstdlib>

visualization_msgs::MarkerConstPtr our_msg;
boost::shared_ptr <tf::TransformListener> tf_listener;

void position_callback(const visualization_msgs::MarkerConstPtr& in_msg);
void handler(int signal_number);

int main(int argn, char* args[])
{
	ros::init(argn, args, "putter_nutter");

	ros::NodeHandle nh;
	
	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = &handler;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTSTP, &sa, NULL);
	
	ros::Subscriber s = nh.subscribe ("/siftobjects", 1, &position_callback);

	tf_listener = boost::shared_ptr <tf::TransformListener> (new tf::TransformListener (ros::Duration (2)));
	
	while(1)
		ros::spinOnce();
		
}

void handler(int signal_number)
{
	if(signal_number == SIGINT)
		printf("Ctrl-C caught...\n");
	else
		printf("Ctrl-Z caught...\n");	
	ros::shutdown();
	
	abort();
}

void position_callback(const visualization_msgs::MarkerConstPtr& in_msg)
{
	geometry_msgs::PointStamped p_final, p;
	
	p.header.stamp = in_msg->header.stamp;
	p.header.frame_id = "LeftCameraLens_link";
	p.point = in_msg->pose.position;
	
	p.point.z = in_msg->pose.position.z - 0.3;
	//p.point.y = -1 * in_msg->pose.position.y ;
	//p.point.z = in_msg->pose.position.x;

	tf_listener->waitForTransform ("base_link", p.header.frame_id, p.header.stamp, ros::Duration(2));
	try
	{
		(*tf_listener).transformPoint ("base_link", p, p_final);
	}
	catch (tf::TransformException& e)
	{
		ROS_INFO ("e.what(): %s", e.what());
		return;
	}	
	std::cout<<p;
	std::cout<<"*********************"<<std::endl;
	std::cout<<p_final;
	std::cout<<"*********************"<<std::endl;
	std::cout<<"*********************"<<std::endl;
	std::cout<<"*********************"<<std::endl;
	our_msg = in_msg;
}
