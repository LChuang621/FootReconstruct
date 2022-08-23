#include "HoleFix.h"

using std::cout;
using std::endl;

// 输入点云foot_coor_flip，输出投影到地面的左右脚踝footupL_projected，footupR_projected
bool CutFootup(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	
	// 直通滤波截取脚踝部分点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud(cloud_in);            //设置输入点云
	//pass.setFilterFieldName("z");         //设置过滤时所需要点云类型的Z字段
	//pass.setFilterLimits(87, 93);        //设置在过滤字段的范围
	//pass.setFilterLimitsNegative(false);   //设置保留范围内还是过滤掉范围内
	////pass.setFilterLimits(0, 90);        //设置在过滤字段的范围
	//pass.setFilterLimitsNegative(false);   //设置保留范围内还是过滤掉范围内
	//pass.filter(*cloud_filtered3);            //执行滤波，保存过滤结果在cloud_filtered

	CutPointCloud(cloud_in, cloud_filtered3, 87, 93, false);

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

	return true;
}

// 输入点云foot_coor_flip，输出投影到地面的左右脚踝footupL_projected，footupR_projected
bool CutPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,int min,int max,bool key)
{

	// 直通滤波截取脚踝部分点云
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_in);            //设置输入点云
	pass.setFilterFieldName("z");         //设置过滤时所需要点云类型的Z字段
	pass.setFilterLimits(min, max);        //设置在过滤字段的范围
	pass.setFilterLimitsNegative(key);   //设置保留范围内还是过滤掉范围内
	pass.filter(*cloud_out);            //执行滤波，保存过滤结果在cloud_filtered

	//pcl::PLYWriter writer;
	//writer.write<pcl::PointXYZ>("foot_cut.ply", *cloud_filtered3, false);

	return true;
}

// 输入点云footupL_projected，输出投影到地面的左右脚踝footup_ret
bool FixFootupHole(pcl::PointCloud<pcl::PointXYZ>::Ptr& footup_projected, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_output, int footup_height)
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


bool FixFootdownHole()
{
	// 下脚底的补齐
#if 1
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
	return true;
}

