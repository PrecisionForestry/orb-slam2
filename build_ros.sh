echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM2

# Clean existing build if present
if [ -d build ]; then
  rm -rf build
fi

mkdir build
cd build

# Use CMake in release mode if CMakeLists supports it (for older ROS use -DROS_BUILD_TYPE)
cmake .. -DROS_BUILD_TYPE=Release

# Detect CPU core count to parallelize make safely
if command -v nproc >/dev/null 2>&1; then
    NUM_CORES=$(nproc)
else
    NUM_CORES=2
fi

make -j${NUM_CORES}
