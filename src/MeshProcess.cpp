#include "MeshProcess.h"

#include <pcl/visualization/pcl_visualizer.h>

using std::cout;
using std::endl;


bool PointCloudSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	// �Ե����ز���  
	pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZ>); // �������������������KD-Tree
	pcl::PointCloud<pcl::PointXYZ> mls_points;   //���MLS
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;  // ������С����ʵ�ֵĶ���mls
	mls.setComputeNormals(false);  //��������С���˼������Ƿ���Ҫ�洢����ķ���
	mls.setInputCloud(cloud_in);        //���ô��������
	mls.setPolynomialOrder(3);             // ���2�׶���ʽ���
	mls.setSearchMethod(treeSampling);    // ����KD-Tree��Ϊ��������
	mls.setSearchRadius(3); // ��λm.����������ϵ�K���ڰ뾶
	mls.process(*cloud_out);        //���

	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("foot_clean_sampling.ply", *cloud_out, false);

	return true;
}



bool TriangularMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	cout << "input Point cloud size : " << cloud_in->points.size() << endl;
	pcl::PCLPointCloud2::Ptr cloud_in2(new pcl::PCLPointCloud2());
	toPCLPointCloud2(*cloud_in, *cloud_in2);

	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_in2);
	sor.setLeafSize(3.0f, 3.0f, 3.0f);
	sor.filter(*cloud_filtered);


	fromPCLPointCloud2(*cloud_filtered, *cloud_out);
	cout << "Scene Point cloud size : " << cloud_out->points.size() << endl;

	pcl::PLYWriter writer;
	writer.write<pcl::PointXYZ>("sampling.ply", *cloud_out, false);


	//�������ҽ���ͶӰ
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudL(new pcl::PointCloud<pcl::PointXYZ>);
	int countL = 0;
	for (int i = 0; i < cloud_out->points.size(); i++)
	{
		pcl::PointXYZ p;
		p.x = cloud_out->points[i].x;
		p.y = cloud_out->points[i].y;
		p.z = cloud_out->points[i].z;


		if (p.y > 0)
		{
			cloudL->points.push_back(p);
			countL++;
		}

	}
	cloudL->width = 1;
	cloudL->height = countL;
	cout << "size is" << cloud_out->size() << endl;
	cout << "L size is" << cloudL->size() << endl;


#if 1
	// ���߹���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;                    //�������߹��ƵĶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);           // ��������ĵ��Ʒ���
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  // �������������������KD-Tree
	tree->setInputCloud(cloudL);                   //��cloud����tree����
	normalEstimation.setInputCloud(cloudL);        //�������
	normalEstimation.setSearchMethod(tree);
	//normalEstimation.setViewPoint(7, 85.7, 45);
	// K����ȷ��������ʹ��k������㣬����ȷ��һ����rΪ�뾶��Բ�ڵĵ㼯��ȷ�������ԣ�����ѡ1����
	normalEstimation.setKSearch(10);                    // ʹ�õ�ǰ����Χ�����10����
	//normalEstimation.setRadiusSearch(0.03);            //����ÿһ���㶼�ð뾶Ϊ3cm�Ľ���������ʽ
	normalEstimation.compute(*normals);                 //���㷨��






	//std::vector<pcl::PointXYZ> pointsl;
	//std::vector<pcl::Normal> normalsl;
	//pointsl.push_back(cloudL->points[0]);
	//normalsl.push_back(normals->points[0]);

	//std::vector<bool> flags(cloudL->size(), false);   //������ǵ����еĵ��Ƿ�������
	//flags[0] = true;

	//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//kdtree.setInputCloud(cloudL);
	//int K = 20;
	//std::vector<int> pointsIdx(K);          //����
	//std::vector<float> pointsDistance(K);   //����

	//while (!normalsl.empty())
	//{
	//	pcl::PointXYZ seed_point = pointsl.back();   //���ӵ�
	//	pcl::Normal seed_normal = normalsl.back();   //���ӵ㷨��
	//	pointsl.pop_back();
	//	normalsl.pop_back();

	//	kdtree.nearestKSearch(seed_point, K, pointsIdx, pointsDistance);    //k��������
	//	Eigen::Vector3f v1(seed_normal.normal_x, seed_normal.normal_y, seed_normal.normal_z);

	//	for (size_t i = 0; i < pointsIdx.size(); i++)
	//	{
	//		if (!flags[pointsIdx[i]])   //����õ�û�б�������
	//		{
	//			Eigen::Vector3f	v2(normals->points[pointsIdx[i]].normal_x,
	//				normals->points[pointsIdx[i]].normal_y,
	//				normals->points[pointsIdx[i]].normal_z);

	//			if (v1.dot(v2) < 0) //����õ㷨�߷��������ӵ㷨�߷����෴���н�Ϊ�۽ǣ�����ת���߷���
	//			{
	//				normals->points[pointsIdx[i]].normal_x *= -1;
	//				normals->points[pointsIdx[i]].normal_y *= -1;
	//				normals->points[pointsIdx[i]].normal_z *= -1;
	//			}
	//			pointsl.push_back(cloudL->points[pointsIdx[i]]);
	//			normalsl.push_back(normals->points[pointsIdx[i]]);
	//			flags[pointsIdx[i]] = true;    //��Ǹõ��Ѿ���������
	//		}
	//	}
	//}

	





#if 0
	//���ӻ�
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	//��ʾ�����뷨�ߣ�2��0.1���Ե������ߵ������볤��
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloudL, normals, 2, 10, "normals");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
#endif

	// ������λ�ˡ���ɫ��������Ϣ���ӵ�һ��
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloudL, *normals, *cloud_with_normals);

	//����������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	//����Poisson���󣬲����ò���
	pcl::Poisson<pcl::PointNormal> pn;
	pn.setConfidence(false); //�Ƿ�ʹ�÷������Ĵ�С��Ϊ������Ϣ�����false�����з���������һ����
	pn.setDegree(2); //���ò���degree[1,5],ֵԽ��Խ��ϸ����ʱԽ�á�
	pn.setDepth(8); //���������ȣ����2^d x 2^d x 2^d������Ԫ�����ڰ˲�������Ӧ�����ܶȣ�ָ��ֵ��Ϊ�����ȡ�
	pn.setIsoDivide(8); //������ȡISO��ֵ����㷨�����
	pn.setManifold(false); //�Ƿ���Ӷ���ε����ģ�����������ǻ�ʱ�� �������б�־���������Ϊtrue����Զ���ν���ϸ�����ǻ�ʱ������ģ�����false�����
	pn.setOutputPolygons(false); //�Ƿ������������񣨶��������ǻ��ƶ�������Ľ����
	pn.setSamplesPerNode(9); //��������һ���˲�������е����������С��������������[1.0-5.0],������[15.-20.]ƽ��
	pn.setScale(1.25); //���������ع���������ֱ���������߽�������ֱ���ı��ʡ�
	pn.setSolverDivide(8); //����������Է������Gauss-Seidel�������������

	//���������������������
	pn.setSearchMethod(tree2);
	pn.setInputCloud(cloud_with_normals);
	//����������������ڴ洢���
	pcl::PolygonMesh mesh;
	//ִ���ع�
	pn.performReconstruction(mesh);

	//��������ͼ
	pcl::io::savePolygonFileSTL("mesh.stl", mesh);














#endif



	return true;
}
