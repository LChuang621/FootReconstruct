#pragma once

#include "FootCommon.h"
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>



bool BoundryExtract(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);