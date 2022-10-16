# Point Cloud Pose Estimation

## Windows Setup

1. Install CUDA from https://developer.nvidia.com/cuda-downloads?target_os=Windows&target_arch=x86_64&target_version=11&target_type=exe_network.

1. Install dependencies with `vcpkg`.

```
vcpkg install pcl[core,cuda,opengl,qt,vtk]:x64-windows --recurse
vcpkg install realsense2:x64-windows
vcpkg install opencv:x64-windows
```

1. Set your vcpkg toolchain file for VSCode if you haven't already:
```json
"cmake.configureSettings": {
    "CMAKE_TOOLCHAIN_FILE": "/home/m/vcpkg/scripts/buildsystems/vcpkg.cmake"
}
```
