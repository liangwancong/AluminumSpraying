#include "Tree.h"

pcl::search::KdTree<pcl::PointXYZ>::Ptr getKdTree(PointCloud::Ptr& cloud) {
	//kdTree ¼ÓËÙËÑË÷
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	return tree;
}
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr getOctree(PointCloud::Ptr& cloud,const float &resolution){
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(resolution));
	octree->setInputCloud(cloud);
	octree->addPointsFromInputCloud();
	return octree;
}