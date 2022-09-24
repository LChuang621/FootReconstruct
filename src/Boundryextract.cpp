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
	//����һ������k���ڲ�ѯ,��ѯ��ʱ�����õ��ڵ����У����һ�����ڵ����䱾��
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
		/*	std::cout << "��ѯ��λ�� " << cloud->points[i] << std::endl;
			std::cout << "����Ϊ�� " << nnh[0] << "  " << nnh[1] << std::endl;
			std::cout << "����Ϊ�� " << cloud->points[nnh[0]] << "  " << cloud->points[nnh[1]] << std::endl;*/

		everagedistance += sqrt(squaredistance[1]);
		//   cout<<everagedistance<<endl;

	}
	everagedistance = everagedistance / (cloud->size() / 2);
	cout << "everage distance is :" << everagedistance << endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //����pcl::PointXYZ��ʾ�����������ݣ�pcl::Normal��ʾ�������,��pcl::Normalǰ�����Ƿ������һ��������
	normEst.setInputCloud(cloud);
	normEst.setSearchMethod(tree);
	//normEst.setRadiusSearch(2);  //������Ƶİ뾶
	normEst.setKSearch(9);  //������Ƶĵ���
	normEst.compute(*normals);
	//cout << "normal size is " << normals->size() << endl;
	//�߽�������Ҫ���� 
	est.setInputCloud(cloud);
	est.setInputNormals(normals);/*M_PI_2 */
	est.setAngleThreshold(M_PI_2);   ///������ ���ڹ��캯���Ѿ���������˳�ʼ�� Ϊ��/2 ���������� ʹ�� M_PI/2  M_PI_2  
	est.setSearchMethod(tree);
	est.setKSearch(100);  //һ���������ֵԽ�ߣ����ձ߽�ʶ��ľ���Խ�� 20+

	//  est.setRadiusSearch(everagedistance);  //�����뾶
	est.compute(boundaries);

	//  pcl::PointCloud<pcl::PointXYZ> boundPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
	int countBoundaries = 0;
	for (int i = 0; i < cloud->size(); i++) {
		uint8_t x = (boundaries.points[i].boundary_point);
		int a = static_cast<int>(x); //�ú����Ĺ�����ǿ������ת��
		if (a == 1)
		{
			//  boundPoints.push_back(cloud->points[i]);
			(*boundPoints).push_back(cloud->points[i]);
			countBoundaries++;
		}
		else
			noBoundPoints.push_back(cloud->points[i]);

	}
	std::cout << "boudary size is��" << countBoundaries << std::endl;
	//  pcl::io::savePCDFileASCII("boudary.pcd",boundPoints);
	//pcl::io::savePCDFileASCII("boudary212.pcd", *boundPoints);
	//pcl::io::savePCDFileASCII("NoBoundpoints.pcd", noBoundPoints);

	pcl::PLYWriter writer2;
	writer2.write<pcl::PointXYZ>("boudary.ply", *boundPoints, false);


	return true;
}
