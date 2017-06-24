#pragma once
//----------------------------【PCL头文件】-------------------------------------
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace pcl;

PointCloud<pcl::PointXYZ>::Ptr scr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
VoxelGrid<pcl::PointXYZ>sor;  //创建滤波对象