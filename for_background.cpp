//
// Created by sun on 19-3-8.
//
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


int main (int argc, char** argv)
{
	// Read in the cloud data
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
	reader.read (argv[1], *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud (cloud);

	float vg_ls =0.01;
	bool vg_leafsizeset = pcl::console::find_switch (argc, argv, "-ls");
	if (vg_leafsizeset)
		pcl::console::parse (argc, argv, "-ls", vg_ls);
	vg.setLeafSize (vg_ls, vg_ls, vg_ls);

	vg.filter (*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

	pcl::PCDWriter writer;

	std::stringstream ss1;

	ss1 << "vg_"  << ".pcd";
	writer.write<pcl::PointXYZ>(ss1.str(), *cloud_filtered, false); //*

	//创建平面模型分割的对象并设置参数
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());


	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);    //分割模型
	seg.setMethodType (pcl::SAC_RANSAC);       //随机参数估计方法

	int s_i =100; //default segMaxIterations;
	float s_t = 1; //default setDistanceThreshold;

	bool s_iset = pcl::console::find_switch (argc, argv, "-si");
	if (s_iset)
		pcl::console::parse (argc, argv, "-si", s_i);

	bool s_tset = pcl::console::find_switch (argc, argv, "-st");
	if (s_tset)
		pcl::console::parse (argc, argv, "-st", s_t);

	seg.setMaxIterations (s_i);                //最大的迭代的次数
	seg.setDistanceThreshold (s_t);            //设置阀值



	int i=0, nr_points = (int) cloud_filtered->points.size ();
	while (cloud_filtered->points.size () > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false); //提取内点

		// Get the points associated with the planar surface
		extract.filter (*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		//  // 移去平面局内点，提取剩余点云
		extract.setNegative (true); //
		extract.filter (*cloud_f);
		*cloud_filtered = *cloud_f;
	}

	std::stringstream ss2;
	ss2 << "planar" << ".pcd";
	writer.write<pcl::PointXYZ>(ss2.str(), *cloud_plane, false); //*

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //欧式聚类对象
	ec.setClusterTolerance (0.2);                       // 设置近邻搜索的搜索半径为2cm
	ec.setMinClusterSize (100);                          //设置一个聚类需要的最少的点数目为100
	ec.setMaxClusterSize (25000);                        //设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod (tree);                           //设置点云的搜索机制
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);                        //从点云中提取聚类，并将点云索引保存在cluster_indices中
	//迭代访问点云索引cluster_indices,直到分割处所有聚类
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)

			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		if  (j == 10||j == 20) {
			ss << "cloud_cluster_" << j << ".pcd";
			writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
		}

		j++;
	}

	return (0);
}
