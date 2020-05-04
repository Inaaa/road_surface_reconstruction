//
// Created by chli on 09.04.20.
//

#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/print.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/console/time.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <iostream>
#include <pcl/surface/impl/organized_fast_mesh.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>


#include <pcl/visualization/common/float_image_utils.h>//保存深度图像
#include <pcl/io/png_io.h>//保存深度图像


#ifndef MASTERARBEIT_RANGE_IMAGE_H
#define MASTERARBEIT_RANGE_IMAGE_H

void range_image(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void range_image2( pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

#endif //MASTERARBEIT_RANGE_IMAGE_H
