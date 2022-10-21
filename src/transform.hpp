#pragma once

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

void CropBox(PointCloudT::Ptr cloud, float min, float max)
{
    pcl::CropBox<PointNT> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(min, min, min, 1.0));
    boxFilter.setMax(Eigen::Vector4f(max, max, max, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*cloud);
}

void Rotate(PointCloudT::Ptr cloud, float degrees)
{
    float radians = degrees * 3.14159 / 180.0;
    Eigen::Matrix4f transformation{
        {cosf(radians), -sinf(radians), 0.0, 0.0},
        {sinf(radians), cosf(radians), 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}};
    pcl::transformPointCloud(*cloud, *cloud, transformation);
}

void ScalePointCloud(PointCloudT::Ptr cloud, float factor)
{
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity() * factor;
    pcl::transformPointCloud(*cloud, *cloud, transformation);
}

PointCloudT::Ptr Load(
    const std::string &path)
{
    // Point clouds
    PointCloudT::Ptr cloud(new PointCloudT);

    // Load object and scene
    pcl::console::print_highlight("Loading point cloud\n");
    if (pcl::io::loadPCDFile<PointNT>(path, *cloud) < 0)
    {
        pcl::console::print_error("Error loading object/scene file!\n");
        return nullptr;
    }
    return cloud;
}

void Transform(const std::string &path, const std::string &output_path)
{
    PointCloudT::Ptr cloud = Load(path);
    // For scene
    CropBox(cloud, -0.5, 0.5);
    ScalePointCloud(cloud, 1000.0);
    Rotate(cloud, 180.0f);
    pcl::io::savePCDFileASCII(output_path, *cloud);
}
