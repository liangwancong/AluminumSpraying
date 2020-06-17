#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "Tree.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void cornerRegistration(PointCloud::Ptr &cornerCloudSource, PointCloud::Ptr &cornerCloudTarget, bool isRotation, Eigen::Matrix4f &tm);
int icpRegistration(PointCloud::Ptr& cloudSource, PointCloud::Ptr& cloudTarget, double &score, Eigen::Matrix4f &tm);
double getIcpScore(PointCloud::Ptr &cloudSource, PointCloud::Ptr &cloudTarget);