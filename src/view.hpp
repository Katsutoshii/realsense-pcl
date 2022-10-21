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
#include <pcl/PolygonMesh.h>
#include "transform.hpp"

// Align a rigid object to a scene with clutter and occlusions
int ViewPCDs(const std::vector<std::string> &paths)
{
    std::vector<PointCloudT::Ptr> clouds;
    for (const auto &path : paths)
    {
        clouds.push_back(Load(path));
    }

    // Show object
    pcl::visualization::PCLVisualizer visu("Viewer");
    std::vector<std::vector<float>> colors = {
        {64.0, 255.0, 64.0},
        {255.0, 64.0, 64.0},
        {64.0, 64.0, 255.0}};
    for (int i = 0; i < clouds.size(); ++i)
    {
        // Hack to rotate the scene.
        if (i == 0)
        {
            Rotate(clouds[i], 180.0f);
        }
        auto color = colors[i % 3];
        visu.addPointCloud(clouds[i], ColorHandlerT(clouds[i], color[0], color[1], color[2]), "cloud" + std::to_string(i));
    }
    visu.spin();

    return 0;
}
