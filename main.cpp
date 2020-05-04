#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "filter.h"
#include "surface.h"
#include "visualization.h"
#include <pcl/visualization/cloud_viewer.h>
#include "range_image.h"
#include <pcl/io/vtk_io.h>
#include "preprocessing.h"

int main ()
{
    //preprocessing();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZ>);
    // load the cloud data
    pcl::io::loadPCDFile ("/home/chli/CLionProjects/masterarbeit/data/0356_3_road_new2.pcd", *cloud);
    radius_filter(cloud, cloud_filtered);



    //std::cout << cloud -> points[0].z;
    //std::cout << cloud_filtered -> points[0].z;

    /* //usual normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    normal_estimation(cloud_filtered, normals, cloud_with_normals );
    */
    //visualization(cloud_filtered);
    voxel_grid(cloud_filtered,cloud_filtered3);

    std::vector<float> vect = road_feature(cloud_filtered3) ;
    std::cout<< "slope=" << *vect.begin()<< std::endl;
    std::cout<< "superelevation=" << *(vect.begin()+1)<< std::endl;

    visualization(cloud_filtered3);


    float rough_slope = slope(cloud_filtered3);
    std::cout << "roght_slope ="<< rough_slope << std::endl;

    bspline_fitting(cloud_filtered3);

    // resamling filter
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>);
    resampling(cloud_filtered3, mls_points);
    //pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);


    //pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZ>("/home/chli/CLionProjects/masterarbeit/data/0356_3_road_befor_voxfilter.pcd", *cloud_filtered, false);
    //writer.write<pcl::PointXYZ>("/home/chli/CLionProjects/masterarbeit/data/0356_3_road_after_voxfilter.pcd", *cloud_filtered, false);

    // greedy triangularation
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    greedy_triangulation(mls_points,triangles);
    //pcl::io::saveVTKFile ("/home/cc/cc_code/masterarbeit/data/0356_3_road.vtk", *triangles);
    //range_image2(cloud_filtered);






    //marchingcubes reconstruction
    //marchingcubes (cloud_with_normals);

    //visualization(cloud_filtered);
    //normalsVis(cloud_filtered, normals);

    //range_image(cloud_filtered);

    return (0);
}


