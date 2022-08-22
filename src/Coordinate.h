#pragma once

#include "FootCommon.h"

#include <pcl/common/impl/transforms.hpp>
#include <pcl/common/impl/common.hpp>



Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f&, Eigen::Vector3f&);

bool GetCenterCoordinate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);