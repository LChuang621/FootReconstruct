// Reconstrcut_test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include <iostream>
#include "FilterGround.h"

using std::cout;
using std::endl;


int main()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string pointcloud = "D:/work/reconstruct/data/origin_fullcloud/cloud_0815.ply";

    cout << pointcloud << endl;
	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(pointcloud, *cloud) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}
	//查看点云数

	cout << "文件共有" << cloud->size() << "个点：" << endl;


    return 0;
}


