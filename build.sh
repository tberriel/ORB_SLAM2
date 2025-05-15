install_path=$1

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release 
make -j
cd ../../

cd g2o
echo "Configuring and building Thirdparty/g2o ..."

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release 
make -j

cd ../../
cd ../
echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=${install_path}
make install -j
