//
// Created by chli on 09.04.20.
//
#include <pcl/visualization/cloud_viewer.h>


#ifndef MASTERARBEIT_VISUALIZATION_H
#define MASTERARBEIT_VISUALIZATION_H


void visualization (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void normalsVis (
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);

#endif //MASTERARBEIT_VISUALIZATION_H
