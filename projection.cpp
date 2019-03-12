//
// Created by sun on 19-3-11.
//

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud output_temp;

int main (int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped (new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1],*cloud))
	{
		pcl::console::print_error("can not loading cloud file\n");
		return (1);
	}

	//设置截取范围
	float z1=0.0;
	bool z1_set = pcl::console::find_switch(argc, argv,"-z1");
	if (z1_set)
		pcl::console::parse(argc,argv,"-z1",z1);
	float z2=0.0;
	bool z2_set = pcl::console::find_switch(argc,argv,"-z2");
	if (z2_set)
		pcl::console::parse(argc,argv,"-z2",z2);

	std::cout<<z1<<":"<<z2<<std::endl;

	pcl::PCDWriter writer;
	std::stringstream ss2;
	ss2 << "cropped" << ".pcd";
	unsigned m=1;

//	std::cout << cloud->sensor_orientation_<<std::endl;
	for (size_t n=0;n<cloud->points.size();n++)
	{

		if (cloud->points[n].z>z1 && cloud->points[n].z<z2)
		{
//			std::cout<<cloud->points[n].z<<"\n";
			cloud_cropped->width = 1;
			cloud_cropped->height = m;
			cloud_cropped->points.push_back(cloud->points[n]);
//			cloud_cropped->points[m].x = cloud->points[n].x;
//			cloud_cropped->points[m].y = cloud->points[n].y;
//			cloud_cropped->points[m].z = cloud->points[n].z;
			m++;
		}
//		std::cout<<m<<"\n";
	}


	writer.write<pcl::PointXYZ>(ss2.str(), *cloud_cropped, false); //*

	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	coefficients->values.resize (4);
	coefficients->values[0] = coefficients->values[1] = 0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 0;

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud_cropped);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);

	std::stringstream ss3;
	ss3 << "projected" << ".pcd";

	writer.write<pcl::PointXYZ>(ss3.str(), *cloud_projected, false);
	return (0);
}