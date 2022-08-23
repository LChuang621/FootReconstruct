#pragma once

#include "FootCommon.h"
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/common.h>

bool CutFootup(pcl::PointCloud<pcl::PointXYZ>::Ptr&, pcl::PointCloud<pcl::PointXYZ>::Ptr&);

bool FixFootupHole(pcl::PointCloud<pcl::PointXYZ>::Ptr&, pcl::PointCloud<pcl::PointXYZ>::Ptr&, int);

bool CutPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr&, pcl::PointCloud<pcl::PointXYZ>::Ptr&, int, int, bool);