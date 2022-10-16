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
#include <pcl/io/auto_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// Align a rigid object to a scene with clutter and occlusions
int convert_stl(const char *filename)
{
    // Point clouds
    PointCloudT::Ptr object(new PointCloudT);
    // Load object and scene
    pcl::console::print_highlight("Converting STL to PCD file...\n");

    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFile("data/nist.stl", mesh) < 0)
    {
        pcl::console::print_error("Error loading STL file!\n");
        return 1;
    }
    std::cout << mesh;
    // pcl::io::savePCDFile("data/nist.pcd", mesh.cloud);

    return 0;
}
