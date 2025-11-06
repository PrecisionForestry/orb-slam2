set -e

echo "Configuring and building Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
if command -v nproc >/dev/null 2>&1; then
    NUM_CORES=$(nproc)
else
    NUM_CORES=2
fi
make -j${NUM_CORES}

cd ../../g2o
echo "Configuring and building Thirdparty/g2o ..."
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
if command -v nproc >/dev/null 2>&1; then
    NUM_CORES=$(nproc)
else
    NUM_CORES=2
fi
make -j${NUM_CORES}

cd ../../../
echo "Uncompress vocabulary ..."
cd Vocabulary
# Change vocabulary to ORBvoc.txt.tar.gz, in case the newly trained ORBvoc.UAV.txt.tar.gz do not work!!!!!!!!!
if [ ! -f ORBvoc.txt ]; then
    tar -xf ORBvoc.UAV.txt.tar.gz
fi


cd ..
echo "Configuring and building ORB_SLAM2 ..."
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
if command -v nproc >/dev/null 2>&1; then
    NUM_CORES=$(nproc)
else
    NUM_CORES=2
fi
make -j${NUM_CORES}
