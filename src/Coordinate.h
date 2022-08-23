#pragma once

#include "FootCommon.h"

#include <pcl/common/impl/transforms.hpp>
#include <pcl/common/impl/common.hpp>

#include <pcl/filters/project_inliers.h>//ͶӰ�˲���ͷ�ļ�

Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f&, Eigen::Vector3f&);

bool GetCenterCoordinate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, pcl::ModelCoefficients::Ptr&, Eigen::Isometry3f);