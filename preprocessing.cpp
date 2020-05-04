//
// Created by chli on 10.04.20.
//

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include "preprocessing.h"

void preprocessing()
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Load bun0.pcd -- should be available with the PCL archive in test

    pcl::io::loadPCDFile("/mrtstorage/users/students/chli/real_data/pcd_fl_2/1571220356.360673000.pcd", *cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());

    for (int i = 0; i < cloud->points.size (); ++i)
    {
        if (cloud->points[i].x != 0)
        {
            cloud1->push_back(cloud->points[i]);

        }
        //std::cout << cloud->points[i] << std::endl;
    }
    //std::cout << cloud->points[500] << std::endl;
    std::cout << cloud->points.size () << std::endl;
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("/mrtstorage/users/chli/real_data/test_data3/1571220356.3.pcd", *cloud, false);
    writer.write<pcl::PointXYZ>("/mrtstorage/users/chli/real_data/test_data3/1571220356.36.pcd", *cloud1);

}