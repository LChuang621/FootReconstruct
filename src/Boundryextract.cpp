#include "Boundryextract.h"

using std::cout;
using std::endl;
using std::vector;




bool BoundryExtract(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < cloud_in->points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = cloud_in->points[i].x;
		p.y = cloud_in->points[i].y;
		p.z = cloud_in->points[i].z;
		cloud->points.push_back(p);
	}
	cloud->width = 1;
	cloud->height = cloud_in->points.size();
	cout << "size is" << cloud->size() << endl;
	
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	int k = 2;
	float everagedistance = 0;
	for (int i = 0; i < cloud->size() / 2; i++)
	{
		//std::cout << "cloud->size()/2" << cloud->points[i] << std::endl;
		vector<int> nnh;
		vector<float> squaredistance;
		//  pcl::PointXYZ p;
		//   p = cloud->points[i];
		kdtree.nearestKSearch(cloud->points[i], k, nnh, squaredistance);
		/*	std::cout << "查询点位： " << cloud->points[i] << std::endl;
			std::cout << "近邻为： " << nnh[0] << "  " << nnh[1] << std::endl;
			std::cout << "近邻为： " << cloud->points[nnh[0]] << "  " << cloud->points[nnh[1]] << std::endl;*/

		everagedistance += sqrt(squaredistance[1]);
		//   cout<<everagedistance<<endl;

	}
	everagedistance = everagedistance / (cloud->size() / 2);
	cout << "everage distance is :" << everagedistance << endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
	normEst.setInputCloud(cloud);
	normEst.setSearchMethod(tree);
	//normEst.setRadiusSearch(2);  //法向估计的半径
	normEst.setKSearch(9);  //法向估计的点数
	normEst.compute(*normals);
	//cout << "normal size is " << normals->size() << endl;
	//边界评估需要点云 
	est.setInputCloud(cloud);
	est.setInputNormals(normals);/*M_PI_2 */
	est.setAngleThreshold(M_PI_2);   ///在这里 由于构造函数已经对其进行了初始化 为Π/2 ，必须这样 使用 M_PI/2  M_PI_2  
	est.setSearchMethod(tree);
	est.setKSearch(100);  //一般这里的数值越高，最终边界识别的精度越好 20+

	//  est.setRadiusSearch(everagedistance);  //搜索半径
	est.compute(boundaries);

	//  pcl::PointCloud<pcl::PointXYZ> boundPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
	int countBoundaries = 0;
	for (int i = 0; i < cloud->size(); i++) {
		uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //该函数的功能是强制类型转换
		if (a == 1)
		{
			//  boundPoints.push_back(cloud->points[i]);
			(*boundPoints).push_back(cloud->points[i]);
			countBoundaries++;
		}
		else
			noBoundPoints.push_back(cloud->points[i]);

	}
	std::cout << "boudary size is：" << countBoundaries << std::endl;
	//  pcl::io::savePCDFileASCII("boudary.pcd",boundPoints);
	//pcl::io::savePCDFileASCII("boudary212.pcd", *boundPoints);
	//pcl::io::savePCDFileASCII("NoBoundpoints.pcd", noBoundPoints);

	pcl::PLYWriter writer2;
	writer2.write<pcl::PointXYZ>("boudary.ply", *boundPoints, false);


	return true;
}
