#include "align.hpp"
#include "view.hpp"
#include "convert_stl.hpp"
#include "realsense.hpp"

int main(int argc, char const *argv[])
{
    return align("data/scene.pcd", "data/nist.pcd");
    // return realsense();
    // return convert_stl("data/nist.stl");
    // return view();
}
