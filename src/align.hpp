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
#include "transform.hpp"

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

void Downsample(PointCloudT::Ptr cloud, float leaf = 0.005f)
{
    // Downsample
    pcl::console::print_highlight("Downsampling...\n");
    pcl::VoxelGrid<PointNT> grid;
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);
}

void EstimateFeatures(PointCloudT::Ptr cloud, FeatureCloudT::Ptr output_features)
{
    // Estimate features
    pcl::console::print_highlight("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch(0.025);
    fest.setInputCloud(cloud);
    fest.setInputNormals(cloud);
    fest.compute(*output_features);
}

// int RunICP(PointCloudT::Ptr object, PointCloudT::Ptr scen)
// {
// }

int RunAlignment(PointCloudT::Ptr object, PointCloudT::Ptr scene, float leaf)
{
    FeatureCloudT::Ptr object_features(new FeatureCloudT);
    FeatureCloudT::Ptr scene_features(new FeatureCloudT);

    // Estimate normals for scene
    pcl::console::print_highlight("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<PointNT, PointNT> nest;
    nest.setRadiusSearch(1.5 * leaf);
    nest.setInputCloud(scene);
    nest.compute(*scene);

    EstimateFeatures(scene, scene_features);
    EstimateFeatures(object, object_features);

    // Perform alignment
    pcl::console::print_highlight("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
    align.setInputSource(object);
    align.setSourceFeatures(object_features);
    align.setInputTarget(scene);
    align.setTargetFeatures(scene_features);
    align.setMaximumIterations(100000);              // Number of RANSAC iterations
    align.setNumberOfSamples(3);                     // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness(5);            // Number of nearest features to use
    align.setSimilarityThreshold(0.25f);             // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance(4.0f * leaf); // Inlier threshold
    align.setInlierFraction(0.1f);                   // Required inlier fraction for accepting a pose hypothesis
    {
        pcl::ScopeTime t("Alignment");
        align.align(*object);
    }

    if (!align.hasConverged())
    {
        pcl::console::print_error("Alignment failed!\n");
        return 1;
    }
    Eigen::Matrix4f transformation = align.getFinalTransformation();

    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
    pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
    pcl::console::print_info("\n");
    // pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());
    return 0;
}

void Display(PointCloudT::Ptr scene, PointCloudT::Ptr object)
{
    vtkObject::GlobalWarningDisplayOff();
    // Show alignment
    pcl::visualization::PCLVisualizer vis("Alignment");
    vis.addCoordinateSystem(5.0, 0.0, 0.0, 0.0);
    vis.addPointCloud(scene, ColorHandlerT(scene, 63.0, 255.0, 63.0), "scene");
    vis.addPointCloud(object, ColorHandlerT(object, 63.0, 63.0, 255.0), "object");

    PCL_INFO("Press q to continue.\n");
    vis.spin();
}

// Align a rigid object to a scene with clutter and occlusions
int Align(const char *scene_pcd, const char *object_pcd)
{
    // Point clouds
    PointCloudT::Ptr scene = Load(scene_pcd);
    PointCloudT::Ptr object = Load(object_pcd);

    const float scale_factor = 0.1;
    ScalePointCloud(object, scale_factor);
    ScalePointCloud(scene, scale_factor);

    const float leaf = 1.5 * scale_factor;
    Downsample(scene, leaf);
    Downsample(object, leaf);
    Display(scene, object);

    RunAlignment(scene, object, leaf);

    // Print results
    printf("\n");
    Display(scene, object);

    // Result
    return 0;
}
