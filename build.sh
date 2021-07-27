# echo "Configuring and building Thirdparty/DBoW3 ..."

# cd Thirdparty/DBoW3
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make

# cd ../../g2o

# echo "Configuring and building Thirdparty/g2o ..."

# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make

# echo "Configuring and building Thirdparty/Pangolin ..."

# cd ../../..

# cd Thirdparty/Pangolin
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make

# cd ../../../ 

# echo "Uncompress vocabulary ..."
# cd Vocabulary
# tar -xf ORBvoc.txt.tar.gz
# cd ..

echo "Configuring and building SuperPoint_SLAM ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

# If compile with multicore, it'will have error
make
