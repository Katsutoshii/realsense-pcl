# Run MSBuild
pushd build
msbuild "./main.sln" /property:Configuration=Release
popd
