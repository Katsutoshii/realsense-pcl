pushd "./build"
cmake build "../src"
msbuild "./alignment_prerejective.sln"
popd
build/Debug/alignment_prerejective.exe "data/chef.pcd" "data/rs1.pcd"
