#include "align.hpp"
#include "view.hpp"
// #include "convert_stl.hpp"
#include "realsense.hpp"
#include "registration.hpp"
#include "transform.hpp"

int main(int argc, char const *argv[])
{
    return Align("data/scaled_scene.pcd", "data/sampled.pcd");
    // Transform("data/scene.pcd", "data/scaled_scene.pcd");
    // ViewPCDs({"data/scaled_scene.pcd", "data/sampled.pcd"});
}
