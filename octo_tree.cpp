//
// Created by sun on 19-3-7.
//

/* * 功能：octree体素内搜索介绍 */
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char** argv)
{
	if (argc<5)
	{
		std::cout << "Usage: " << "<Resolution> " << "<SearchPoint.x>" << "<SearchPoint.y>" << "<SearchPoint.z>" << std::endl;
		return -2;
	}
	else
		cout << "Resolution : " << argv[1] << endl;
	//// 构造一个1m X 1m X 1m立方体点云，每个点之间间隔1cm
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pCubeCloud(new pcl::PointCloud<pcl::PointXYZ>);
	//for (int x = 0; x < 100; x++)
	//{
	// for (int y = 0; y < 100; y++)
	// {
	// for (int z = 0; z < 100; z++)
	// {
	// // 注意这里是pCubeCloud->push_back，而不是pCubeCloud->points->push_back
	// pCubeCloud->push_back(pcl::PointXYZ((float)x/100.0f, (float)y/100.0f, (float)z/100.0f));
	// }
	// }
	//}
	//// save
	//pcl::io::savePCDFile("cube.pcd", *pCubeCloud, true);

	// load
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCube(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile("cube.pcd", *pCube) == -1)
		return -1;

	// 初始化八叉树
	float resolution = atof(argv[1]);                                          // 设置体素大小????
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
	//octree.defineBoundingBox(0, 0, 0, 1, 1, 1);
	// 将点云载入八叉树
	octree.setInputCloud(pCube);
	octree.addPointsFromInputCloud();

	// 初始化查询点
	pcl::PointXYZ searchPoint(atof(argv[2]), atof(argv[3]), atof(argv[4]));    // 查询点

	// 体素内搜索
	std::vector<int> pointIdxVec;
	pcl::PointCloud<pcl::RGB>::Ptr pPointsRGB(new pcl::PointCloud<pcl::RGB>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pCloudShow(new pcl::PointCloud<pcl::PointXYZRGBA>);


	if (octree.voxelSearch(searchPoint, pointIdxVec))
	{
		std::cout << "Neighbors within voxel search at (" << searchPoint.x
		          << " " << searchPoint.y
		          << " " << searchPoint.z << ")"
		          << std::endl;

		std::cout << "Searched Points Number : " << pointIdxVec.size() << endl;

		std::cout << "Leaf Count : " << octree.getLeafCount() << std::endl;             // 叶子数
		std::cout << "Tree Depth : " << octree.getTreeDepth() << std::endl;             // 八叉树深度
		std::cout << "Branch Count : " << octree.getBranchCount() << std::endl;         // 非叶子结点数
		std::cout << "Voxel Diameter : " << octree.getVoxelSquaredDiameter() << std::endl;  // ???Voxel Side Length*3
		std::cout << "Voxel Side Length : " << octree.getVoxelSquaredSideLen() << std::endl;// 分辨率的平方
		double minx, miny, minz, maxx, maxy, maxz;
		octree.getBoundingBox(minx, miny, minz, maxx, maxy, maxz);
		std::cout << "BoundingBox: " <<  "(" << minx << " - " << maxx << ")" << " , " << "(" << miny << " - " << maxy << ")" << " , " << "(" << minz << " - " << maxz << ")" << std::endl;                                // 整个八叉树的范围


	}

	// 给搜索到的点上色，原始点云中的点全为蓝色，搜索到的上为红色
	pPointsRGB->width = pCube->size();
	pPointsRGB->height = 1;
	pPointsRGB->resize(pPointsRGB->width*pPointsRGB->height);

	pCloudShow->width = pCube->size();
	pCloudShow->height = 1;
	pCloudShow->resize(pPointsRGB->width*pPointsRGB->height);

	for (size_t i = 0; i < pPointsRGB->size(); i++)
	{
		pPointsRGB->points[i].b = 255;
	}

	for (size_t i = 0; i < pointIdxVec.size(); ++i)
	{
		pPointsRGB->points[pointIdxVec[i]].b = 0;
		pPointsRGB->points[pointIdxVec[i]].r = 255;
	}

	// 合并不同字段
	pcl::concatenateFields(*pCube, *pPointsRGB, *pCloudShow);

	// 可视窗口初始化
	pcl::visualization::PCLVisualizer viewer ("octotree Viewer");
	viewer.setCameraFieldOfView(0.785398);      // fov 大概45度
	viewer.setBackgroundColor(0.0, 0.0, 0.0);   // 背景设为灰色
	viewer.setCameraPosition(
			0, 0, 5,                                // camera位置
			0, 0, -1,                               // view向量--相机朝向
			0, 1, 0                                 // up向量
	);
	viewer.addPointCloud(pCloudShow,"Out");
	viewer.addCoordinateSystem(1.0, "cloud", 0);
	while (!viewer.wasStopped()) { // 显示，直到‘q’键被按下
		viewer.spinOnce();
	}

	//pcl::io::savePLYFileBinary("voxelSearchResult.ply", *pCloudShow); // 用cloudcompare读入时rgba变为0
	system("pause");
	return 0;
}