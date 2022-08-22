#include "Coordinate.h"


using std::cout;
using std::endl;
using std::vector;



Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f& before, Eigen::Vector3f& after)
{
    // ref blog.csdn.net/zhazhiqiang/article/details/52441170
    before.normalize();
    after.normalize();

    float angle = acos(before.dot(after));
    Eigen::Vector3f p_rotate = before.cross(after);
    p_rotate.normalize();

    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
    rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle)) - p_rotate[2] * sin(angle);
    rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));


    rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));


    rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));

    return rotationMatrix;
}




bool GetCenterCoordinate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    
    double mean;	//点云均值
    double stddev;	//点云标准差
    vector<float> vec_x,vec_y;
    for (auto i=0; i< cloud->size(); i++)
    {
        vec_x.emplace_back(cloud->points[i].x);
        vec_y.emplace_back(cloud->points[i].y);
    }

    pcl::getMeanStd(vec_x, mean, stddev);
    cout <<"x mean is " << mean << endl;

    pcl::getMeanStd(vec_y, mean, stddev);
    cout << "y mean is " << mean << endl;
    return true;
}