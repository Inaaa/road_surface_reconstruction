//
// Created by chli on 09.04.20.
//
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/visualization/pcl_visualizer.h>//可视化
#include <pcl/surface/marching_cubes_hoppe.h>// 移动立方体算法
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/common/pca.h>

#include <time.h>
#include <math.h>
#include <typeinfo>


#ifndef MASTERARBEIT_SURFACE_H
#define MASTERARBEIT_SURFACE_H

void greedy_triangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals,
        pcl::PolygonMesh::Ptr triangles);
void marchingcubes (pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);

void bspline_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

std::vector<float> road_feature (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

#endif //MASTERARBEIT_SURFACE_H
