#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

pcl::search::KdTree<pcl::PointXYZ>::Ptr getKdTree(PointCloud::Ptr& cloud);
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr getOctree(PointCloud::Ptr& cloud, const float &resolution);