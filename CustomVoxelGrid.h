/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2020, Chanchan Li
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
#ifndef MASTERARBEIT_CUSTOMVOXELGRID_H
#define MASTERARBEIT_CUSTOMVOXELGRID_H
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <boost/sort/spreadsort/integer_sort.hpp>

using Array4size_t = Eigen::Array<std::size_t, 4, 1>;

template <typename PointT>
class CustomVoxelGrid: public pcl::VoxelGrid<PointT> {
    using pcl::Filter<PointT>::getClassName;
    using PointCloud = typename pcl::Filter<PointT>::PointCloud;
    using pcl::VoxelGrid<PointT>::input_;
    using pcl::VoxelGrid<PointT>::filter_field_name_;
    using pcl::VoxelGrid<PointT>::indices_;
    using pcl::VoxelGrid<PointT>::filter_limit_min_;
    using pcl::VoxelGrid<PointT>::filter_limit_max_;
    using pcl::VoxelGrid<PointT>::filter_limit_negative_;
    using pcl::VoxelGrid<PointT>::inverse_leaf_size_;
    using pcl::VoxelGrid<PointT>::min_b_;
    using pcl::VoxelGrid<PointT>::max_b_;
    using pcl::VoxelGrid<PointT>::div_b_;
    using pcl::VoxelGrid<PointT>::divb_mul_;
    using pcl::VoxelGrid<PointT>::min_points_per_voxel_;
    using pcl::VoxelGrid<PointT>::save_leaf_layout_;
    using pcl::VoxelGrid<PointT>::leaf_layout_;
    using pcl::VoxelGrid<PointT>::downsample_all_data_;

    void applyFilter (PointCloud &output) override;
};



#endif //MASTERARBEIT_CUSTOMVOXELGRID_H
