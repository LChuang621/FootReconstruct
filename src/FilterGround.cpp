// Reconstrcut_test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "FilterGround.h"
#include "Coordinate.h"
#include "Boundryextract.h"

using std::cout;
using std::endl;


void DisplayPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
}



bool FilterGround(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_foot, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_ground)
{
	if (cloud->size() == 0)
	{
		PCL_ERROR("Point cloud is null.");
		return false;
	}
	cout << "file total has " << cloud->size() << " points..." << endl;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	// 距离阈值 单位m
	seg.setDistanceThreshold(8);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	cout << " a is " << coefficients->values[0] << endl
		<< " b is " << coefficients->values[1] << endl
		<< " c is " << coefficients->values[2] << endl
		<< " d is " << coefficients->values[3] << endl;

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return false;
	}

	// 提取地面
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.filter(*cloud_ground);

	//// 提取脚
	extract.setNegative(true);
	extract.filter(*cloud_foot);


	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::ProjectInliers<pcl::PointXYZRGB> proj;
	//proj.setModelType(pcl::SACMODEL_PLANE);
	//proj.setInputCloud(cloud_foot);
	//proj.setModelCoefficients(coefficients);
	//proj.filter(*cloud_projected);
	//pcl::PLYWriter writer2;
	//writer2.write<pcl::PointXYZRGB>("foot_Project.ply", *cloud_projected, false);

	return true;
}



bool DownSampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_down, float LeafSize)
{
	if (cloud->size() == 0)
	{
		PCL_ERROR("Point cloud is null.");
		return false;
	}
	// 输出降采样前的点云数量
	cout << "points quantity before voxel downsample:" << cloud->size() << endl;


	//创建ApproximateVexelGrid体素降采样对象
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> vg;
	vg.setInputCloud(cloud);
	vg.setLeafSize(LeafSize, LeafSize, LeafSize);  //设置体素大小为0.15m x 0.15m x 0.15m
	vg.filter(*cloud_down);

	//输出降采样后的点云数量
	std::cout << "points quantity after voxex downsample:" << cloud_down->size() << std::endl;

	//对比显示降采样效果
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	int v1(0);  //创建左窗口显示原始点云
	viewer.createViewPort(0, 0, 0.5, 1.0, v1);  //左右窗口大小划分，1:1
	viewer.setBackgroundColor(0, 0, 0, v1);
	viewer.addText("Before Downsample", 2, 2, "Before Downsample", v1);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> rgb1(cloud, "z");
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb1, "cloud before downsample", v1);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud before downsample", v1);
	viewer.addCoordinateSystem(1.0, "Before Voxel Downsample", v1);
	int v2(1);  //创建右窗口显示降采样后的点云
	viewer.createViewPort(0.5, 0, 1.0, 1.0, v2);  //左右窗口大小划分，1:1
	viewer.setBackgroundColor(0, 0, 0, v2);
	viewer.addText("After Downsample", 2, 2, "After Downsample", v2);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> rgb2(cloud_down, "z");
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud_down, rgb2, "cloud after downsample", v2);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud after downsample", v2);
	viewer.addCoordinateSystem(1.0, "After Voxel Downsample", v2);

	viewer.spin();


	return true;
}



bool FootOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_clean)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

	// 统计滤波
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50); //设置在进行统计时考虑查询点邻近点数
	sor.setStddevMulThresh(1.0); //设置判断是否为离群点的阈值
	sor.filter(*cloud_filtered);

	// 半径滤波
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;  //创建滤波器
	outrem.setInputCloud(cloud_filtered);    //设置输入点云
	outrem.setRadiusSearch(8);    //设置半径为8的范围内找临近点
	outrem.setMinNeighborsInRadius(50);//设置查询点的邻域点集数小于50的删除
	outrem.filter(*cloud_clean);  //执行条件滤波

	pcl::PLYWriter writer2;
	writer2.write<pcl::PointXYZRGB>("foot_clean.ply", *cloud_clean, false);

	return true;
}


bool FootBoundryOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_clean)
{
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setRadiusSearch(8);
	sor.setMinNeighborsInRadius(20);
	sor.setNegative(false);
	sor.filter(*cloud_clean);
	std::cout << "boundPoints_fliter size is:" << cloud_clean->size() << endl;
	pcl::PLYWriter writer2;
	writer2.write<pcl::PointXYZ>("footboudary_fliter.ply", *cloud_clean, false);
	return true;
}



int main()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_foot(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr foot_lean(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string pointcloud = "D:/work/reconstruct/data/origin_fullcloud/foot_clean.ply";
#if 1

	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(pointcloud, *cloud) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}

#else
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud, *cloud2) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}
#endif

	//FilterGround(cloud, cloud_foot, cloud_ground); // 分离地面和双脚
	//FootOutlierRemoval(cloud_foot, foot_lean); // 双脚离群点滤波
	//BoundryExtract(foot_lean, cloud_filtered); // 提取边界
#if 0
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZRGB>("ground.pcd", *cloud_ground, false);
	writer.write<pcl::PointXYZRGB>("foot.pcd", *cloud_foot, false);
#endif

#if 0
	// 计算相机坐标系到地面坐标系的变换矩阵
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	// 创建一个系数为X=Y=0,Z=1的平面
	coefficients->values.resize(4);
	coefficients->values[0] = 0.969502;
	coefficients->values[1] = 0.000401399;
	coefficients->values[2] = 0.245082;
	coefficients->values[3] = -104.598;
	Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
	GetCenterCoordinate(cloud, coefficients, T);
#endif




	//FootBoundryOutlierRemoval(cloud2, cloud_filtered2);


    return 0;
}


