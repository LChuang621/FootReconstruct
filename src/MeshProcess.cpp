#include "MeshProcess.h"

#include <pcl/visualization/pcl_visualizer.h>

using std::cout;
using std::endl;


bool PointCloudSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
{
	// 对点云重采样  
	pcl::search::KdTree<pcl::PointXYZ>::Ptr treeSampling(new pcl::search::KdTree<pcl::PointXYZ>); // 创建用于最近邻搜索的KD-Tree
	pcl::PointCloud<pcl::PointXYZ> mls_points;   //输出MLS
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;  // 定义最小二乘实现的对象mls
	mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线
	mls.setInputCloud(cloud_in);        //设置待处理点云
	mls.setPolynomialOrder(3);             // 拟合2阶多项式拟合
	mls.setSearchMethod(treeSampling);    // 设置KD-Tree作为搜索方法
	mls.setSearchRadius(3); // 单位m.设置用于拟合的K近邻半径
	mls.process(*cloud_out);        //输出

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


	//分离左右脚踝投影
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
	// 法线估计
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;                    //创建法线估计的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);           // 定义输出的点云法线
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);  // 创建用于最近邻搜索的KD-Tree
	tree->setInputCloud(cloudL);                   //用cloud构造tree对象
	normalEstimation.setInputCloud(cloudL);        //输入点云
	normalEstimation.setSearchMethod(tree);
	//normalEstimation.setViewPoint(7, 85.7, 45);
	// K近邻确定方法，使用k个最近点，或者确定一个以r为半径的圆内的点集来确定都可以，两者选1即可
	normalEstimation.setKSearch(10);                    // 使用当前点周围最近的10个点
	//normalEstimation.setRadiusSearch(0.03);            //对于每一个点都用半径为3cm的近邻搜索方式
	normalEstimation.compute(*normals);                 //计算法线






	//std::vector<pcl::PointXYZ> pointsl;
	//std::vector<pcl::Normal> normalsl;
	//pointsl.push_back(cloudL->points[0]);
	//normalsl.push_back(normals->points[0]);

	//std::vector<bool> flags(cloudL->size(), false);   //用来标记点云中的点是否被生长过
	//flags[0] = true;

	//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//kdtree.setInputCloud(cloudL);
	//int K = 20;
	//std::vector<int> pointsIdx(K);          //索引
	//std::vector<float> pointsDistance(K);   //距离

	//while (!normalsl.empty())
	//{
	//	pcl::PointXYZ seed_point = pointsl.back();   //种子点
	//	pcl::Normal seed_normal = normalsl.back();   //种子点法线
	//	pointsl.pop_back();
	//	normalsl.pop_back();

	//	kdtree.nearestKSearch(seed_point, K, pointsIdx, pointsDistance);    //k近邻搜索
	//	Eigen::Vector3f v1(seed_normal.normal_x, seed_normal.normal_y, seed_normal.normal_z);

	//	for (size_t i = 0; i < pointsIdx.size(); i++)
	//	{
	//		if (!flags[pointsIdx[i]])   //如果该点没有被生长到
	//		{
	//			Eigen::Vector3f	v2(normals->points[pointsIdx[i]].normal_x,
	//				normals->points[pointsIdx[i]].normal_y,
	//				normals->points[pointsIdx[i]].normal_z);

	//			if (v1.dot(v2) < 0) //如果该点法线方向与种子点法线方向相反（夹角为钝角），则翻转法线方向
	//			{
	//				normals->points[pointsIdx[i]].normal_x *= -1;
	//				normals->points[pointsIdx[i]].normal_y *= -1;
	//				normals->points[pointsIdx[i]].normal_z *= -1;
	//			}
	//			pointsl.push_back(cloudL->points[pointsIdx[i]]);
	//			normalsl.push_back(normals->points[pointsIdx[i]]);
	//			flags[pointsIdx[i]] = true;    //标记该点已经被生长过
	//		}
	//	}
	//}

	





#if 0
	//可视化
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	//显示点云与法线，2和0.1可以调整法线的疏密与长短
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloudL, normals, 2, 10, "normals");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
#endif

	// 将点云位姿、颜色、法线信息连接到一起
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloudL, *normals, *cloud_with_normals);

	//创建搜索树
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	//创建Poisson对象，并设置参数
	pcl::Poisson<pcl::PointNormal> pn;
	pn.setConfidence(false); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
	pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。
	pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
	pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度
	pn.setManifold(false); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
	pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）
	pn.setSamplesPerNode(9); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
	pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。
	pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度

	//设置搜索方法和输入点云
	pn.setSearchMethod(tree2);
	pn.setInputCloud(cloud_with_normals);
	//创建多变形网格，用于存储结果
	pcl::PolygonMesh mesh;
	//执行重构
	pn.performReconstruction(mesh);

	//保存网格图
	pcl::io::savePolygonFileSTL("mesh.stl", mesh);














#endif



	return true;
}
