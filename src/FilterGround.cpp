// Reconstrcut_test.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "FilterGround.h"
#include "Coordinate.h"
#include "Boundryextract.h"
#include "MeshProcess.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include "HoleFix.h"



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
	seg.setDistanceThreshold(5);
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

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloud_clean);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;//创建欧式聚类分割对象
	ec.setClusterTolerance(3); //设置近邻搜索的搜索半径
	ec.setMinClusterSize(50); //设置最小聚类尺寸
	ec.setMaxClusterSize(150000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_clean);
	ec.extract(cluster_indices);


	int max = -1;
	int max2 = -1;
	int index = -1;
	int index2 = -1;
	int index0 = 0;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> Eucluextra; //用于储存欧式分割后的点云
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(cloud_clean->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		Eucluextra.push_back(cloud_cluster);

		//cout << cloud_cluster->size() << endl;
		if (max <= max2) {
			if (int(cloud_cluster->size()) > max)
			{
				max = cloud_cluster->size();
				index = index0;
			}
				
		}
		else
		{
			if (int(cloud_cluster->size()) > max2)
			{
				max2 = cloud_cluster->size();
				index2 = index0;
			}
				
		}
		index0++;
	}


	//*cloud_clean = *Eucluextra[index] + *Eucluextra[index2];

	pcl::PLYWriter writer2;
	writer2.write<pcl::PointXYZRGB>("foot_clean.ply", *Eucluextra[index] + *Eucluextra[index2], false);

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


// 输入点云footupL_projected，输出投影到地面的左右脚踝footup_ret
bool FootupFill(pcl::PointCloud<pcl::PointXYZ>::Ptr& footup_projected, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_output,int footup_height)
{
	// 调整点云，让点云落在xy坐标系的第一象限(x>0,y>0)，生成新的点云
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*footup_projected, minPt, maxPt);
	cout << "footup min.x = " << minPt.x << "  " << "footup min.y = " << minPt.y << "  " << "footup min.z = " << minPt.z << endl;
	cout << "footup max.x = " << maxPt.x << "  " << "footup max.y = " << maxPt.y << "  " << "footup max.z = " << maxPt.z << endl;


	// 调整后的点云cloud_adjusted
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_adjusted(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < footup_projected->points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = footup_projected->points[i].x - minPt.x;
		p.y = footup_projected->points[i].y - minPt.y;
		p.z = footup_projected->points[i].z;

		cloud_adjusted->points.push_back(p);
	}
	cloud_adjusted->width = 1;
	cloud_adjusted->height = footup_projected->points.size();


#if 0
	// 保存调整后的点云
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("footupNL_projected.ply", *cloud_adjusted, false);
#endif


	// 将投影点云转换成二值化图像rgbimg
	pcl::PointXYZ minPt_adjusted, maxPt_adjusted;
	pcl::getMinMax3D(*cloud_adjusted, minPt_adjusted, maxPt_adjusted);
	cout << "footup adjusted min.x = " << minPt_adjusted.x << "  " << "footup adjusted min.y = " << minPt_adjusted.y << endl;
	cout << "footup adjusted max.x = " << maxPt_adjusted.x << "  " << "footup adjusted max.y = " << maxPt_adjusted.y << endl;

	cv::Mat footup_img(maxPt_adjusted.y + 1 + 10, maxPt_adjusted.x + 1 + 10, CV_8UC1, cv::Scalar(255));
	for (int i = 0; i < cloud_adjusted->points.size(); i++)
	{
		int cols, rows;
		cols = cloud_adjusted->points[i].x;
		rows = cloud_adjusted->points[i].y;

		footup_img.at<bool>(cloud_adjusted->points[i].y + 5, cloud_adjusted->points[i].x + 5) = 0;

	}


	// 形态学处理，使断开的轮廓变成连通轮廓
	cv::Mat element, erode_rgb, dilate_rgb;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Mat kernel = (cv::Mat_<float>(1, 2) << 1, 1);
	kernel.convertTo(element, CV_8UC1);
	cv::erode(footup_img, erode_rgb, element);
	cv::dilate(erode_rgb, dilate_rgb, element);

	// 生成轮廓并筛选
	findContours(dilate_rgb, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
	int thresh = std::max(maxPt_adjusted.y + 1 + 10, maxPt_adjusted.x + 1 + 10) * 1.6;
	cout << "thresh = " << thresh << endl;
	int min = std::max(maxPt_adjusted.y + 1 + 10, maxPt_adjusted.x + 1 + 10) * 5;
	int idx;

	for (int i = 0; i < contours.size(); i++)
	{
		cout << contours[i].size() << endl;


		if (contours[i].size() > thresh)
		{
			if (contours[i].size() < min)
			{
				min = contours[i].size();
				idx = i;
			}
		}
	}
	cout << "min is " << min << endl;


	// 绘出并填充轮廓
	cv::Mat drawing = cv::Mat::zeros(dilate_rgb.size(), CV_8UC1);
	drawContours(drawing, contours, idx, cv::Scalar(255), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
	cv::floodFill(drawing, cv::Point(0, 0), cv::Scalar(255));


	// 填充好的2d点重新投影到3d,并生成点云
	cout << "rows is " << drawing.rows << "cols is " << drawing.cols << endl;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudret(new pcl::PointCloud<pcl::PointXYZ>);
	int count = 0;
	for (int rows = 0; rows < drawing.rows; rows++)
	{
		for (int cols = 0; cols < drawing.cols; cols++)
		{
			if (drawing.at<bool>(rows, cols) == 0)
			{
				pcl::PointXYZ p;
				p.x = cols - 5 + minPt.x;
				p.y = rows - 5 + minPt.y;
				p.z = 90;
				cloud_output->points.push_back(p);
				count++;
			}
		}
	}
	cloud_output->width = 1;
	cloud_output->height = count;

	//writer.write<pcl::PointXYZ>("footup_R.ply", *cloudret, false);
	return true;
}


bool GetBoundry(cv::Mat ground_img, cv::Mat range_img, cv::Mat& img_ret)
{

	cv::Mat down_xor, down_diff;
	cv::bitwise_xor(ground_img, range_img, down_xor);
	cv::bitwise_not(down_xor, down_diff);

	cv::Mat foot_not, footboundry_and;
	cv::bitwise_not(ground_img, foot_not);

	cv::Mat element, dilate_rgb;
	cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, 1, 0, 1, 1, 1, 0, 1, 0);
	kernel.convertTo(element, CV_8UC1);

	cv::Mat foot_dilate;
	cv::dilate(foot_not, foot_dilate, element);
	cv::bitwise_not(foot_dilate, foot_dilate);
	cv::bitwise_xor(down_xor, foot_dilate, footboundry_and);
	cv::bitwise_xor(footboundry_and, range_img, footboundry_and);
	cv::bitwise_not(footboundry_and, img_ret);

	return true;
}



void Producepointcloud(cv::Mat& img, pcl::PointXYZ& minPt, std::string save_path, int height)
{
	cv::Mat drawing;
	int count;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudreturn(new pcl::PointCloud<pcl::PointXYZ>);
	 drawing = img.clone();
	cloudreturn->clear();
	count = 0;
	for (int rows = 0; rows < drawing.rows; rows++)
	{
		for (int cols = 0; cols < drawing.cols; cols++)
		{
			if (drawing.at<bool>(rows, cols) == 0)
			{
				pcl::PointXYZ p;
				p.x = cols - 5 + minPt.x;
				p.y = rows - 5 + minPt.y;
				p.z = height;
				cloudreturn->points.push_back(p);
				count++;
			}
		}
	}
	cloudreturn->width = 1;
	cloudreturn->height = count;


	// 保存调整后的点云
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>(save_path, *cloudreturn, false);
}


int main()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_foot(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr foot_clean(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string pointcloud = "D:/work/reconstruct/data/origin_fullcloud_5mm/cloud_0815.ply";


#if 0

	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(pointcloud, *cloud) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}

#else
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(pointcloud, *cloud2) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}
#endif

#if 0
	// cloud路径origin_fullcloud/cloud_0815.ply
	FilterGround(cloud, cloud_foot, cloud_ground); // 分离地面和双脚
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZRGB>("ground.ply", *cloud_ground, false);
	writer.write<pcl::PointXYZRGB>("foot.ply", *cloud_foot, false);
#endif

#if 0
	// cloud路径origin_fullcloud_5/foot.ply
	FootOutlierRemoval(cloud, foot_clean); // 双脚离群点滤波
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZRGB>("foot_clean2.ply", *foot_clean, false);
#endif

	// cloud路径foot_coor_flip.ply
	//CutFootup(cloud2, cloud_filtered2); // 1.裁剪上脚踝 2.将点云投影到地面

#if 0
	// cloud路径foot_coor_flip.ply
	FixFootupHole(cloud2, cloud_filtered2, 0); // 调整点云位置，转成rgb图像，填充并生成点云
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("footup_R.ply", *cloud_filtered2, false);
#endif

#if 0
	// cloud路径foot_coor_flip.ply
	CutPointCloud(cloud2, cloud_filtered2, 0, 90, false); // 裁剪保留z方向0-90mm范围内的点云
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("foot_cut.ply", *cloud_filtered2, false);
#endif
	

	//BoundryExtract(cloud, cloud_filtered); // 提取边界
	//FootBoundryOutlierRemoval(cloud2, cloud_filtered2);
	//FootupFill



	//PointCloudSampling(cloud2, cloud_filtered2);
	//TriangularMesh(cloud2, cloud_filtered2);


#if 0
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("foot_clean_sampling.ply", *cloud_filtered2, false);
#endif







#if 0

	// 调整坐标系流程
	// 1.计算旋转矩阵T,作用于foot_clean，得到foot_clean_T
	// 2.绕z轴负方向旋转41°,绕y方向旋转180°,得到点云foot_clean_flip

	// 输入foot_clean,打印旋转矩阵.
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

	
#if 0
	// 输入点云foot_coor_flip，输出投影到地面的左右脚踝footupL_projected，footupR_projected
	// 直通滤波截取脚踝部分点云
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud2);            //设置输入点云
	pass.setFilterFieldName("z");         //设置过滤时所需要点云类型的Z字段
	//pass.setFilterLimits(87, 93);        //设置在过滤字段的范围
	//pass.setFilterLimitsNegative(false);   //设置保留范围内还是过滤掉范围内
	pass.setFilterLimits(0, 90);        //设置在过滤字段的范围
	pass.setFilterLimitsNegative(false);   //设置保留范围内还是过滤掉范围内
	pass.filter(*cloud_filtered3);            //执行滤波，保存过滤结果在cloud_filtered

	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("foot_cut.ply", *cloud_filtered3, false);

	// 投影到地面
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	coefficients->values.resize(4);
	coefficients->values[0] = 0;
	coefficients->values[1] = 0;
	coefficients->values[2] = 1;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud_filtered3);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);


	pcl::PLYWriter writer2;
	writer2.write<pcl::PointXYZ>("footup_projected.ply", *cloud_projected, false);

	//分离左右脚踝投影
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudL(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudR(new pcl::PointCloud<pcl::PointXYZ>);
	int countL = 0;
	int countR = 0;
	for (int i = 0; i < cloud_projected->points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = cloud_projected->points[i].x;
		p.y = cloud_projected->points[i].y;
		p.z = cloud_projected->points[i].z;
		

		if (p.y > 0)
		{
			cloudL->points.push_back(p);
			countL++;
		}
		else
		{
			cloudR->points.push_back(p);
			countR++;
		}

	}
	cloudL->width = 1;
	cloudL->height = countL;
	cloudR->width = 1;
	cloudR->height = countR;
	cout << "size is" << cloud_projected->size() << endl;
	cout << "L size is" << cloudL->size() << endl;
	cout << "R size is" << cloudR->size() << endl;

	writer2.write<pcl::PointXYZ>("footupL_projected.ply", *cloudL, false);
	writer2.write<pcl::PointXYZ>("footupR_projected.ply", *cloudR, false);

#endif



#if 0
	//分离左右脚底投影,输入foot_project_flip
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudL(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudR(new pcl::PointCloud<pcl::PointXYZ>);
	int countL = 0;
	int countR = 0;
	for (int i = 0; i < cloud2->points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = cloud2->points[i].x;
		p.y = cloud2->points[i].y;
		p.z = cloud2->points[i].z;


		if (p.y > 0)
		{
			cloudL->points.push_back(p);
			countL++;
		}
		else
		{
			cloudR->points.push_back(p);
			countR++;
		}

	}
	cloudL->width = 1;
	cloudL->height = countL;
	cloudR->width = 1;
	cloudR->height = countR;
	cout << "size is" << cloud2->size() << endl;
	cout << "L size is" << cloudL->size() << endl;
	cout << "R size is" << cloudR->size() << endl;

	pcl::PLYWriter writer2;
	writer2.write<pcl::PointXYZ>("footdownL_projected.ply", *cloudL, false);
	writer2.write<pcl::PointXYZ>("footdownR_projected.ply", *cloudR, false);
#endif


#if 0


	// 输入footdownR_projected/footdownL_projected，输出
	
	pointcloud = "D:/work/reconstruct/data/origin_fullcloud/footdownR_projected.ply";


#if 0

	if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(pointcloud, *cloud) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}

#else
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(pointcloud, *cloud2) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}
#endif
	
	// 调整点云，让点云落在xy坐标系的第一象限(x>0,y>0)，生成新的点云
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloud2, minPt, maxPt);
	cout << "footup min.x = " << minPt.x << "  " << "footup min.y = " << minPt.y << "  " << "footup min.z = " << minPt.z << endl;
	cout << "footup max.x = " << maxPt.x << "  " << "footup max.y = " << maxPt.y << "  " << "footup max.z = " << maxPt.z << endl;


	// 调整后的点云cloud_adjusted
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_adjusted(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < cloud2->points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = cloud2->points[i].x - minPt.x;
		p.y = cloud2->points[i].y - minPt.y;
		p.z = cloud2->points[i].z;

		cloud_adjusted->points.push_back(p);
	}
	cloud_adjusted->width = 1;
	cloud_adjusted->height = cloud2->points.size();


#if 0
	// 保存调整后的点云
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("footdownL_adjusted.ply", *cloud_adjusted, false);
#endif


	// 将投影点云转换成二值化图像rgbimg
	pcl::PointXYZ minPt_adjusted, maxPt_adjusted;
	pcl::getMinMax3D(*cloud_adjusted, minPt_adjusted, maxPt_adjusted);
	cout << "footup adjusted min.x = " << minPt_adjusted.x << "  " << "footup adjusted min.y = " << minPt_adjusted.y << endl;
	cout << "footup adjusted max.x = " << maxPt_adjusted.x << "  " << "footup adjusted max.y = " << maxPt_adjusted.y << endl;

	cv::Mat footup_img(maxPt_adjusted.y + 1 + 10, maxPt_adjusted.x + 1 + 10, CV_8UC1, cv::Scalar(255));
	for (int i = 0; i < cloud_adjusted->points.size(); i++)
	{
		int cols, rows;
		cols = cloud_adjusted->points[i].x;
		rows = cloud_adjusted->points[i].y;

		footup_img.at<bool>(cloud_adjusted->points[i].y + 5, cloud_adjusted->points[i].x + 5) = 0;

	}

#else
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud, *cloud2) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}
#endif

	// 形态学处理，使断开的轮廓变成连通轮廓
	cv::Mat element, erode_rgb, dilate_rgb;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Mat kernel = (cv::Mat_<float>(4, 4) << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
	kernel.convertTo(element, CV_8UC1);
	//cv::erode(footup_img, erode_rgb, element);
	//cv::dilate(erode_rgb, dilate_rgb, element);
	dilate_rgb = footup_img.clone();
	cv::Mat dilate_rgb2 = dilate_rgb.clone();
	cv::floodFill(dilate_rgb2, cv::Point(0, 0), cv::Scalar(0));
	cv::bitwise_not(dilate_rgb2, dilate_rgb2);
	cv::bitwise_and(dilate_rgb2, dilate_rgb, dilate_rgb);

	cv::Mat dilate2;
	cv::erode(dilate_rgb, dilate2, element);


	cv::Mat drawing = dilate2.clone();
	// 填充好的2d点重新投影到3d,并生成点云
	cout << "rows is " << drawing.rows << "cols is " << drawing.cols << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudret(new pcl::PointCloud<pcl::PointXYZ>);
	int count = 0;
	for (int rows = 0; rows < drawing.rows; rows++)
	{
		for (int cols = 0; cols < drawing.cols; cols++)
		{
			if (drawing.at<bool>(rows, cols) == 0)
			{
				pcl::PointXYZ p;
				p.x = cols - 5 + minPt.x;
				p.y = rows - 5 + minPt.y;
				p.z = 0;
				cloudret->points.push_back(p);
				count++;
			}
		}
	}
	cloudret->width = 1;
	cloudret->height = count;

#if 1
	// 保存调整后的点云
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("footdown_R.ply", *cloudret, false);
#endif


#endif


	// 下脚底的补齐
#if 0
	std::string ground_path = "D:/work/reconstruct/data/origin_fullcloud/cloud_ground_flip.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground2(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(ground_path, *cloud_ground2) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}

	std::string footL_path = "D:/work/reconstruct/data/origin_fullcloud/footdown_L.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_footL(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(footL_path, *cloud_footL) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}

	// 调整点云，让点云落在xy坐标系的第一象限(x>0,y>0)，生成新的点云
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloud_footL, minPt, maxPt);
	cout << "footup min.x = " << minPt.x << "  " << "footup min.y = " << minPt.y << "  " << "footup min.z = " << minPt.z << endl;
	cout << "footup max.x = " << maxPt.x << "  " << "footup max.y = " << maxPt.y << "  " << "footup max.z = " << maxPt.z << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudret(new pcl::PointCloud<pcl::PointXYZ>);
	int count = 0;
	for (int i = 0; i < cloud_ground2->points.size(); i++)
	{
		int cols, rows;
		cols = cloud_ground2->points[i].x;
		rows = cloud_ground2->points[i].y;

		if (minPt.x <= cols && cols <= maxPt.x && minPt.y <= rows && rows <= maxPt.y)
		{
			pcl::PointXYZ p;
			p.x = cols;
			p.y = rows;
			p.z = cloud_ground2->points[i].z;
			cloudret->points.push_back(p);
			count++;
		}

	}
	cloudret->width = 1;
	cloudret->height = count;
#if 0
	// 保存调整后的点云
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("footdownR_temp.ply", *cloudret, false);
#endif

	// 调整点云，让点云落在xy坐标系的第一象限(x>0,y>0)，生成新的点云
	pcl::PointXYZ GminPt, GmaxPt;
	pcl::getMinMax3D(*cloudret, GminPt, GmaxPt);
	cout << "footup min.x = " << GminPt.x << "  " << "footup min.y = " << GminPt.y << "  " << "footup min.z = " << GminPt.z << endl;
	cout << "footup max.x = " << GmaxPt.x << "  " << "footup max.y = " << GmaxPt.y << "  " << "footup max.z = " << GmaxPt.z << endl;


	// 调整后的点云cloud_adjusted
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_adjusted(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < cloudret->points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = cloudret->points[i].x - GminPt.x;
		p.y = cloudret->points[i].y - GminPt.y;
		p.z = cloudret->points[i].z;

		cloud_adjusted->points.push_back(p);
	}
	cloud_adjusted->width = 1;
	cloud_adjusted->height = cloudret->points.size();

	// 调整后的点云cloud_adjusted
	pcl::PointCloud<pcl::PointXYZ>::Ptr foot_adjusted(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < cloud_footL->points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = cloud_footL->points[i].x - GminPt.x;
		p.y = cloud_footL->points[i].y - GminPt.y;
		p.z = cloud_footL->points[i].z;

		foot_adjusted->points.push_back(p);
	}
	foot_adjusted->width = 1;
	foot_adjusted->height = cloudret->points.size();



#if 0
	// 保存调整后的点云
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("footdownR_temp_adjusted.ply", *cloud_adjusted, false);
#endif

	// 将投影点云转换成二值化图像rgbimg
	pcl::PointXYZ minPt_adjusted, maxPt_adjusted;
	pcl::getMinMax3D(*cloud_adjusted, minPt_adjusted, maxPt_adjusted);
	cout << "footup adjusted min.x = " << minPt_adjusted.x << "  " << "footup adjusted min.y = " << minPt_adjusted.y << endl;
	cout << "footup adjusted max.x = " << maxPt_adjusted.x << "  " << "footup adjusted max.y = " << maxPt_adjusted.y << endl;

	cv::Mat ground_img(maxPt_adjusted.y + 1 + 10, maxPt_adjusted.x + 1 + 10, CV_8UC1, cv::Scalar(255));
	for (int i = 0; i < cloud_adjusted->points.size(); i++)
	{
		int cols, rows;
		cols = cloud_adjusted->points[i].x;
		rows = cloud_adjusted->points[i].y;

		ground_img.at<bool>(cloud_adjusted->points[i].y + 5, cloud_adjusted->points[i].x + 5) = 0;

	}

	cv::Mat footdown_img(maxPt_adjusted.y + 1 + 10, maxPt_adjusted.x + 1 + 10, CV_8UC1, cv::Scalar(255));
	for (int i = 0; i < foot_adjusted->points.size(); i++)
	{
		int cols, rows;
		cols = foot_adjusted->points[i].x;
		rows = foot_adjusted->points[i].y;

		footdown_img.at<bool>(foot_adjusted->points[i].y + 5, foot_adjusted->points[i].x + 5) = 0;

	}



	cv::Mat footdown_not, ground_and, element, erode_rgb, erode_rgb2, erode_rgb3, erode_rgb4, erode_ret;
	cv::Mat erode_not, ground_and2;
	cv::bitwise_not(footdown_img, footdown_not);
	cv::bitwise_and(ground_img, footdown_not, ground_and);

	cv::Mat kernel = (cv::Mat_<float>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);
	kernel.convertTo(element, CV_8UC1);
	cv::erode(ground_and, erode_rgb, element);
	cv::erode(erode_rgb, erode_rgb2, element);
	cv::erode(erode_rgb2, erode_rgb3, element);

	kernel = (cv::Mat_<float>(4, 4) << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
	kernel.convertTo(element, CV_8UC1);
	cv::erode(erode_rgb2, erode_rgb4, element);
	cv::erode(erode_rgb4, erode_ret, element);


	cv::bitwise_not(erode_ret, erode_not);
	
	cv::Mat dilate_rgb, dilate_rgb2;
	kernel = (cv::Mat_<float>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);
	kernel.convertTo(element, CV_8UC1);
	cv::erode(erode_not, dilate_rgb, element);
	cv::erode(dilate_rgb, dilate_rgb2, element);

	cv::bitwise_and(ground_and, dilate_rgb2, ground_and2);





	cv::Mat drawing = dilate_rgb2.clone();
	// 填充好的2d点重新投影到3d,并生成点云
	cout << "rows is " << drawing.rows << "cols is " << drawing.cols << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudreturn(new pcl::PointCloud<pcl::PointXYZ>);
	count = 0;
	for (int rows = 0; rows < drawing.rows; rows++)
	{
		for (int cols = 0; cols < drawing.cols; cols++)
		{
			if (drawing.at<bool>(rows, cols) == 0)
			{
				pcl::PointXYZ p;
				p.x = cols - 5 + GminPt.x;
				p.y = rows - 5 + GminPt.y;
				p.z = 0;
				cloudreturn->points.push_back(p);
				count++;
			}
		}
	}
	cloudreturn->width = 1;
	cloudreturn->height = count;

#if 1
	// 保存调整后的点云
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("footdown_L_ret.ply", *cloudreturn, false);
#endif

#endif

#if 0

	std::string footrange_path = "D:/work/reconstruct/data/origin_fullcloud/footdown_L.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_footrange(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(footrange_path, *cloud_footrange) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}

	// 调整点云，让点云落在xy坐标系的第一象限(x>0,y>0)，生成新的点云
	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D(*cloud_footrange, minPt, maxPt);
	cout << "footup min.x = " << minPt.x << "  " << "footup min.y = " << minPt.y << "  " << "footup min.z = " << minPt.z << endl;
	cout << "footup max.x = " << maxPt.x << "  " << "footup max.y = " << maxPt.y << "  " << "footup max.z = " << maxPt.z << endl;

	//std::string footL_path = "D:/work/reconstruct/data/origin_fullcloud/footdown_L_ret2.ply";
	std::string footL_path = "D:/work/reconstruct/data/origin_fullcloud/footdown_L_ret2.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_footL(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(footL_path, *cloud_footL) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}

	//// 调整点云，让点云落在xy坐标系的第一象限(x>0,y>0)，生成新的点云
	//pcl::PointXYZ GminPt, GmaxPt;
	//pcl::getMinMax3D(*cloud_footL, GminPt, GmaxPt);
	//cout << "footup min.x = " << GminPt.x << "  " << "footup min.y = " << GminPt.y << "  " << "footup min.z = " << GminPt.z << endl;
	//cout << "footup max.x = " << GmaxPt.x << "  " << "footup max.y = " << GmaxPt.y << "  " << "footup max.z = " << GmaxPt.z << endl;

	// 调整后的点云cloud_adjusted
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_adjusted(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < cloud_footL->points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = cloud_footL->points[i].x - minPt.x;
		p.y = cloud_footL->points[i].y - minPt.y;
		p.z = cloud_footL->points[i].z;

		cloud_adjusted->points.push_back(p);
	}
	cloud_adjusted->width = 1;
	cloud_adjusted->height = cloud_footL->points.size();


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudrange_adjusted(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < cloud_footrange->points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = cloud_footrange->points[i].x - minPt.x;
		p.y = cloud_footrange->points[i].y - minPt.y;
		p.z = cloud_footrange->points[i].z;

		cloudrange_adjusted->points.push_back(p);
	}
	cloudrange_adjusted->width = 1;
	cloudrange_adjusted->height = cloud_footrange->points.size();



	// 将投影点云转换成二值化图像rgbimg
	pcl::PointXYZ minPt_adjusted, maxPt_adjusted;
	pcl::getMinMax3D(*cloudrange_adjusted, minPt_adjusted, maxPt_adjusted);
	cout << "footup adjusted min.x = " << minPt_adjusted.x << "  " << "footup adjusted min.y = " << minPt_adjusted.y << endl;
	cout << "footup adjusted max.x = " << maxPt_adjusted.x << "  " << "footup adjusted max.y = " << maxPt_adjusted.y << endl;

	cv::Mat ground_img(maxPt_adjusted.y + 1 + 10, maxPt_adjusted.x + 1 + 10, CV_8UC1, cv::Scalar(255));
	for (int i = 0; i < cloud_adjusted->points.size(); i++)
	{
		int cols, rows;
		cols = cloud_adjusted->points[i].x;
		rows = cloud_adjusted->points[i].y;

		ground_img.at<bool>(cloud_adjusted->points[i].y + 5, cloud_adjusted->points[i].x + 5) = 0;

	}

	cv::Mat range_img(maxPt_adjusted.y + 1 + 10, maxPt_adjusted.x + 1 + 10, CV_8UC1, cv::Scalar(255));
	for (int i = 0; i < cloudrange_adjusted->points.size(); i++)
	{
		int cols, rows;
		cols = cloudrange_adjusted->points[i].x;
		rows = cloudrange_adjusted->points[i].y;

		range_img.at<bool>(cloudrange_adjusted->points[i].y + 5, cloudrange_adjusted->points[i].x + 5) = 0;

	}

	cv::Mat ret1, ret2, ret3, ret4, ret5, ret6, ret7, ret8;
	cv::Mat temp;

	GetBoundry(ground_img, range_img, ret1);
	temp = ret1.clone();
	cv::floodFill(temp, cv::Point(temp.size().width/2, temp.size().height / 2), cv::Scalar(0));

	GetBoundry(temp, range_img, ret2);
	temp = ret2.clone();
	cv::floodFill(temp, cv::Point(temp.size().width / 2, temp.size().height / 2), cv::Scalar(0));

	GetBoundry(temp, range_img, ret3);
	temp = ret3.clone();
	cv::floodFill(temp, cv::Point(temp.size().width / 2, temp.size().height / 2), cv::Scalar(0));

	GetBoundry(temp, range_img, ret4);
	temp = ret4.clone();
	cv::floodFill(temp, cv::Point(temp.size().width / 2, temp.size().height / 2), cv::Scalar(0));

	GetBoundry(temp, range_img, ret5);
	temp = ret5.clone();
	cv::floodFill(temp, cv::Point(temp.size().width / 2, temp.size().height / 2), cv::Scalar(0));

	GetBoundry(temp, range_img, ret6);
	temp = ret6.clone();
	cv::floodFill(temp, cv::Point(temp.size().width / 2, temp.size().height / 2), cv::Scalar(0));

	GetBoundry(temp, range_img, ret7);
	temp = ret7.clone();
	cv::floodFill(temp, cv::Point(temp.size().width / 2, temp.size().height / 2), cv::Scalar(0));

	GetBoundry(temp, range_img, ret8);
	temp = ret8.clone();
	cv::floodFill(temp, cv::Point(temp.size().width / 2, temp.size().height / 2), cv::Scalar(0));


	Producepointcloud(ret1, minPt, "footdown_L_1.ply", 1);
	Producepointcloud(ret2, minPt, "footdown_L_2.ply", 2);
	Producepointcloud(ret3, minPt, "footdown_L_3.ply", 3);
	Producepointcloud(ret4, minPt, "footdown_L_4.ply", 4);
	Producepointcloud(ret5, minPt, "footdown_L_5.ply", 5);
	Producepointcloud(ret6, minPt, "footdown_L_6.ply", 6);
	Producepointcloud(ret7, minPt, "footdown_L_7.ply", 7);
	Producepointcloud(ret8, minPt, "footdown_L_8.ply", 8);



#if 0
	// 填充好的2d点重新投影到3d,并生成点云
	cout << "rows is " << drawing.rows << "cols is " << drawing.cols << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudreturn(new pcl::PointCloud<pcl::PointXYZ>);
	int count = 0;
	for (int rows = 0; rows < drawing.rows; rows++)
	{
		for (int cols = 0; cols < drawing.cols; cols++)
		{
			if (drawing.at<bool>(rows, cols) == 0)
			{
				pcl::PointXYZ p;
				p.x = cols - 5 + minPt.x;
				p.y = rows - 5 + minPt.y;
				p.z = 2;
				cloudreturn->points.push_back(p);
				count++;
			}
		}
	}
	cloudreturn->width = 1;
	cloudreturn->height = count;


	// 保存调整后的点云
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("footdown_R_2.ply", *cloudreturn, false);
#endif



#endif




#if 0

	//std::string footL_path = "D:/work/reconstruct/data/origin_fullcloud/footdown_L_ret2.ply";
	std::string footL_path = "D:/work/reconstruct/data/origin_fullcloud/footdown_L_1.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_footL(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(footL_path, *cloud_footL) == -1)
	{
		PCL_ERROR("Read file fail!\n");
		return -1;
	}

	// 调整点云，让点云落在xy坐标系的第一象限(x>0,y>0)，生成新的点云
	pcl::PointXYZ GminPt, GmaxPt;
	pcl::getMinMax3D(*cloud_footL, GminPt, GmaxPt);
	cout << "footup min.x = " << GminPt.x << "  " << "footup min.y = " << GminPt.y << "  " << "footup min.z = " << GminPt.z << endl;
	cout << "footup max.x = " << GmaxPt.x << "  " << "footup max.y = " << GmaxPt.y << "  " << "footup max.z = " << GmaxPt.z << endl;

	// 调整后的点云cloud_adjusted
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_adjusted(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < cloud_footL->points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = cloud_footL->points[i].x - GminPt.x;
		p.y = cloud_footL->points[i].y - GminPt.y;
		p.z = cloud_footL->points[i].z;

		cloud_adjusted->points.push_back(p);
	}
	cloud_adjusted->width = 1;
	cloud_adjusted->height = cloud_footL->points.size();

	// 将投影点云转换成二值化图像rgbimg
	pcl::PointXYZ minPt_adjusted, maxPt_adjusted;
	pcl::getMinMax3D(*cloud_adjusted, minPt_adjusted, maxPt_adjusted);
	cout << "footup adjusted min.x = " << minPt_adjusted.x << "  " << "footup adjusted min.y = " << minPt_adjusted.y << endl;
	cout << "footup adjusted max.x = " << maxPt_adjusted.x << "  " << "footup adjusted max.y = " << maxPt_adjusted.y << endl;

	cv::Mat ground_img(maxPt_adjusted.y + 1 + 10, maxPt_adjusted.x + 1 + 10, CV_8UC1, cv::Scalar(255));
	for (int i = 0; i < cloud_adjusted->points.size(); i++)
	{
		int cols, rows;
		cols = cloud_adjusted->points[i].x;
		rows = cloud_adjusted->points[i].y;

		ground_img.at<bool>(cloud_adjusted->points[i].y + 5, cloud_adjusted->points[i].x + 5) = 0;

	}

	cv::floodFill(ground_img, cv::Point(0, 0), cv::Scalar(0));
	cv::bitwise_not(ground_img, ground_img);

	cv::Mat element, dilate_rgb;
	cv::Mat kernel = (cv::Mat_<float>(2, 2) << 1, 1, 1, 1);
	kernel.convertTo(element, CV_8UC1);

	cv::erode(ground_img, dilate_rgb, element);

	//cv::Mat drawing = dilate_rgb.clone();

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	// 生成轮廓并筛选
	findContours(dilate_rgb, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

	//cv::Mat drawing = cv::Mat::zeros(dilate_rgb.size(), CV_8UC1);
	//int index = 0;
	//for (; index < contours.size(); index ++)
	//{
	//	cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
	//	drawContours(drawing, contours, index, cv::Scalar(255), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
	//}

	int thresh = std::max(maxPt_adjusted.y + 1 + 10, maxPt_adjusted.x + 1 + 10) * 1.6;
	cout << "thresh = " << thresh << endl;
	int min = std::max(maxPt_adjusted.y + 1 + 10, maxPt_adjusted.x + 1 + 10) * 5;
	int idx;

	for (int i = 0; i < contours.size(); i++)
	{
		cout << contours[i].size() << endl;


		if (contours[i].size() > thresh)
		{
			if (contours[i].size() < min)
			{
				min = contours[i].size();
				idx = i;
			}
		}
	}
	cout << "min is " << min << endl;


	// 绘出并填充轮廓
	cv::Mat drawing = cv::Mat::zeros(dilate_rgb.size(), CV_8UC1);
	drawContours(drawing, contours, idx, cv::Scalar(255), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());


	cv::bitwise_not(drawing, drawing);


	// 填充好的2d点重新投影到3d,并生成点云
	cout << "rows is " << drawing.rows << "cols is " << drawing.cols << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudreturn(new pcl::PointCloud<pcl::PointXYZ>);
	int count = 0;
	for (int rows = 0; rows < drawing.rows; rows++)
	{
		for (int cols = 0; cols < drawing.cols; cols++)
		{
			if (drawing.at<bool>(rows, cols) == 0)
			{
				pcl::PointXYZ p;
				p.x = cols - 5 + GminPt.x;
				p.y = rows - 5 + GminPt.y;
				p.z = 4;
				cloudreturn->points.push_back(p);
				count++;
			}
		}
	}
	cloudreturn->width = 1;
	cloudreturn->height = count;

#if 1
	// 保存调整后的点云
	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("footdown_R_2.ply", *cloudreturn, false);
#endif


#endif




    return 0;
}
