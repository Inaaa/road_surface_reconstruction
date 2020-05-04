//
// Created by chli on 09.04.20.
//
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include "surface.h"
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include <pcl/surface/on_nurbs/triangulation.h>

#ifndef MASTERARBEIT_BSPLINE_H
#define MASTERARBEIT_BSPLINE_H

void bspline(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


#endif //MASTERARBEIT_BSPLINE_H
