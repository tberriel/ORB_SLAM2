if [ $# -eq 0 ]
then
  install_path=$CONDA_PREFIX
else
  install_path=$1
fi
BUILD_MODE="Release"
echo "Configuring and building Thirdparty/DBoW2 ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

cd Thirdparty/DBoW2
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE="Release"
make -j
cd ../../

cd g2o
echo "Configuring and building Thirdparty/g2o ..."

mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="../install" \
         -DCMAKE_BUILD_TYPE="Release" 
make install -j

cd ../../
cd ../
echo "Uncompress vocabulary ..."

echo "Configuring and building ORB_SLAM2 ..."

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=$BUILD_MODE  \
         -DCMAKE_INSTALL_PREFIX=${install_path}
make install -j
