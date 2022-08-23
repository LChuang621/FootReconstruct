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




bool GetCenterCoordinate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::ModelCoefficients::Ptr& coefficient, Eigen::Isometry3f Trans)
{
    Eigen::Vector3f groud_normal, camera_normal;
    groud_normal << coefficient->values[0], coefficient->values[1], coefficient->values[2];
    camera_normal << 0, 0, 1;
    Eigen::Matrix4f transformed = CreateRotateMatrix(groud_normal, camera_normal);
    //cout <<"transdormed 1 =" << std::endl << std::fixed << transformed.matrix() << endl;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficient);
    proj.filter(*cloud_projected);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_projected, centroid);
    //std::cout << "地面中心坐标（"
    //    << centroid[0] << ","
    //    << centroid[1] << ","
    //    << centroid[2] << ")." << std::endl;

    Eigen::Matrix3f key;

    key << 1.0f, 0.0f, 0.0f,

        0.0f, 1.0f, 0.0f,

        0.0f, 0.0f, 1.0f;
    Eigen::Vector3f t = Eigen::Vector3f(-centroid[0], -centroid[1], -centroid[2]);

    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    
    Eigen::AngleAxisf rotation_vector;
    rotation_vector.fromRotationMatrix(key);
    T.rotate(rotation_vector);
    T.pretranslate(t);

    Trans = transformed * T.matrix();
    cout << "transdormed =" << std::endl << std::fixed << Trans.matrix() << endl;

    return true;
}