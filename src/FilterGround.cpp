// Reconstrcut_test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include "FilterGround.h"

using std::cout;
using std::endl;


int main()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string pointcloud = "D:/work/reconstruct/data/origin_fullcloud/cloud_0815.ply";

    cout << pointcloud << endl;
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(pointcloud, *cloud) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}
	//查看点云数

	cout << "文件共有" << cloud->size() << "个点：" << endl;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;

	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	// 距离阈值 单位m
	seg.setDistanceThreshold(8);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return (-1);
	}

	// 提取地面
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.filter(*cloud_filtered);

	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGB>("3dpoints_ground.pcd", *cloud_filtered, false);
#if 0
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud_filtered, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
#endif


	// 提取除地面外的物体
	extract.setNegative(true);
	extract.filter(*cloud_filtered);

	writer.write<pcl::PointXYZRGB>("3dpoints_object.pcd", *cloud_filtered, false);

    return 0;
}


