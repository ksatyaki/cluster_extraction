#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/shared_ptr.hpp>
#include <doro_msgs/Cluster.h>
#include <iostream>

int main()
{
	boost::shared_ptr <int> ssk;
	
	for(int i = 0; i < 20; i++)
	{
		pcl::PointXYZ pt;
		pt.x = i;
		pt.y = i;
		pt.z = i;

		pcloud.push_back(pt);
	}

	pcloud.width = 4;
	pcloud.height = 5;
	pcloud.is_dense = true;

	for(int i = 0; i < 5; i++)
		for(int j = 0; j < 4; j++)
	{
		std::cout<<std::endl;
		std::cout<<pcloud.at(j,i)<<" == "<<pcloud[i*pcloud.width + j];

		int index = i*pcloud.width + j;

		std::cout<<"index  "<<index<<"  pcloud.width"<<pcloud.width<<"   ";

		double x = index % pcloud.width;
		double y = (double) (index - x) / (double) pcloud.width;

		std::cout<<"\t"<<"("<<y<<","<<x<<")"<<"\t";
		std::cout<<"\t"<<"("<<i<<","<<j<<")"<<"\t";

		sleep(1);

	}

}
