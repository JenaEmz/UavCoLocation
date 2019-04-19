# 需要g2o和DBoW2
echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2/
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

catkin build

source ${HOME}/catkin_ws/devel/setup.bash
# roslaunch orb_formation orb.launch
