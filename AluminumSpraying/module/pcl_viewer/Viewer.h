#pragma once
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void printUsage(const char* progName);
std::string print4x4Matrix(const Eigen::Matrix4d & matrix);
void print3x3Matrix(const Eigen::Matrix3d & matrix, const char * name);
boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer();
void viewerCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, PointCloud::Ptr &cloud, const int &R, const int &G, const int &B, const int &num, const int &size);
void viewerCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, PointCloud &cloud, const int &R, const int &G, const int &B, const int &num, const int &size);
void viewerArrow(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const PointT &xp, const PointT &yp, const PointT &zp, const PointT &op, const int &color_mode);
void viewerArrow(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const Eigen::Vector3f &v_x, const Eigen::Vector3f &v_y, const Eigen::Vector3f &v_z, const Eigen::Vector3f &oop, const int & color_mode);
void viewBox(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, const Eigen::Vector4f &tm, const Eigen::Vector3f &center, const Eigen::Vector3f &whd, const int &num);
void viewerStop(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer);


