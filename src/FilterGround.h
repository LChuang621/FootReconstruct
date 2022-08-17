#pragma once

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>


void DisplayPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

bool DownSampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr, float);

bool FilterGround(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

bool FootOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);