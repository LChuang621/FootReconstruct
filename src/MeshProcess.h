#pragma once

#include "FootCommon.h"
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_lib_io.h>

#include <pcl/filters/voxel_grid.h>

bool PointCloudSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr&, pcl::PointCloud<pcl::PointXYZ>::Ptr&);

bool TriangularMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);