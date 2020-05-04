//
// Created by chli on 09.04.20.
//
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>

#include "filter.h"
#include "CustomVoxelGrid.cpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>


void radius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius (5);
    // apply filter
    outrem.filter (*cloud_filtered);

}

void passthrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.3, 0.3);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

}







void voxel_grid (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{

    //std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
     //         << " data points (" << pcl::getFieldsList (*cloud) << ").";

    // Create the filtering object
    CustomVoxelGrid<pcl::PointXYZ> customVoxelGrid;
    customVoxelGrid.setInputCloud (cloud);
    customVoxelGrid.setLeafSize (0.01f, 0.01f, 0.1f);
    customVoxelGrid.getMinBoxCoordinates();
    customVoxelGrid.filter (*cloud_filtered);

    //std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
      //        << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."<< std::endl;

    //pcl::PCDWriter writer;
    //writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered,
    //              Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

}

void resampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        pcl::PointCloud<pcl::PointNormal>::Ptr mls_points)
{
    //std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
     //         << " data points (" << pcl::getFieldsList (*cloud) << ").";
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.3);

    // Reconstruct
    mls.process (*mls_points);
    //std::cerr << "PointCloud after filtering: " << mls_points->width * mls_points->height
      //        << " data points (" << pcl::getFieldsList (*mls_points) << ")."<<std::endl;
}


void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                       pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_filtered);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    //tree->setInputCloud (cloud_filtered);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.1);
    ne.setKSearch (20);
    ne.compute (*normals);


}

void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                       pcl::PointCloud<pcl::Normal>::Ptr normals,
                       pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_filtered);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    //tree->setInputCloud (cloud_filtered);
    ne.setSearchMethod (tree);
    //ne.setRadiusSearch (0.1);
    ne.setKSearch (20);
    ne.compute (*normals);
    // Concatenate the XYZ and normal fields*

    pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

}



void normal_estimation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                       pcl::PointCloud<pcl::Normal>::Ptr normals_small_scale,
                       pcl::PointCloud<pcl::Normal>::Ptr normals_large_scale)
{
    // normal estimation
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_filtered);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    ne.setRadiusSearch (0.1);
    ne.compute (*normals_small_scale);

    // Output datasets
    ne.setRadiusSearch (0.3);
    ne.compute (*normals_large_scale);
}

float slope(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-0.2, 0.2);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setLeafSize (3.0f, 2.0f, 0.1f);
    sor.getMinBoxCoordinates();
    sor.filter (*cloud_filtered2);

    std::vector <float> x;
    std::vector <float> z;

    int size = cloud_filtered2->size();
    for( int i = 0; i < size;i++)
    {
        //std::cout<< cloud_filtered2->points[i] << std::endl;
        x.push_back(cloud_filtered2->points[i].x);
        z.push_back(cloud_filtered2->points[i].z);
    }

    std::vector<float>::iterator biggest = std::max_element(std::begin(z), std::end(z));
    auto h = std::distance(std::begin(z), biggest);
    float biggest_x = x.at(h);
    auto smallest = std::min_element(std::begin(z), std::end(z));
    float smallest_x = x.at(std::distance(std::begin(z), smallest));

    //std::cout << *biggest<< " and big_x " << biggest_x << std::endl;
    //std::cout << *smallest<< " and small_x " << smallest_x << std::endl;

    float slope = -(*biggest-*smallest)/(biggest_x-smallest_x)*100;
    //std::cout << slope << std::endl;
    return slope;


}





/*
// Create output cloud for DoN results
pcl::PointCloud<pcl::Normal>::Ptr doncloud (new pcl::PointCloud<pcl::Normal>);
copyPointCloud (*cloud, *doncloud);


//生成DoN分割器
pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::Normal, pcl::Normal> don;
don.setInputCloud (cloud);
don.setNormalScaleLarge (normals_large_scale);
don.setNormalScaleSmall (normals_small_scale);

//计算法线差
don.computeFeature (*doncloud);

// Build the condition for filtering
pcl::ConditionOr<pcl::Normal>::Ptr range_cond (
        new pcl::ConditionOr<pcl::Normal> ()
);
range_cond->addComparison (pcl::FieldComparison<pcl::Normal>::ConstPtr (
        new pcl::FieldComparison<pcl::Normal> ("curvature", pcl::ComparisonOps::GT, 5))

// Build the filter
pcl::ConditionalRemoval<pcl::Normal> condrem;
condrem.setCondition (range_cond);
condrem.setInputCloud (doncloud);

pcl::PointCloud<pcl::Normal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::Normal>);

// Apply filter
condrem.filter (*doncloud_filtered);

doncloud = doncloud_filtered;

// Save filtered output
std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false);
 */