cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


message("Build type: " ${CMAKE_BUILD_TYPE})
option(BUILD_PYTHON_BINDINGS "Build Python bindings for ORB_SLAM2" ON)
message(STATUS "Build Python bindings: ${BUILD_PYTHON_BINDINGS}")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -O3 -march=native)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
)

target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/Examples/RGB-D)

add_executable(rgbd_tum
Examples/RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/Examples/Stereo)

add_executable(stereo_kitti
Examples/Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti ${PROJECT_NAME})

add_executable(stereo_euroc
Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc ${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/Examples/Monocular)

add_executable(mono_tum
Examples/Monocular/mono_tum.cc)
target_link_libraries(mono_tum ${PROJECT_NAME})
#
add_executable(mono_kitti
Examples/Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti ${PROJECT_NAME})

add_executable(mono_euroc
Examples/Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc ${PROJECT_NAME})

# ---------------
# Installation instructions.
# Will install to CMAKE_INSTALL_PREFIX
# This defaults to /usr/local/, specify -DCMAKE_INSTALL_PREFIX when calling cmake to specify an alternate location
# e.g. cmake -DCMAKE_INSTALL_PREFIX="~/local"
# ---------------

install(TARGETS ${PROJECT_NAME}
    DESTINATION  lib
    COMPONENT library)
install(FILES
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    DESTINATION lib
    COMPONENT library)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
    DESTINATION include/ORB_SLAM2
    COMPONENT library
    FILES_MATCHING PATTERN "*.h"
    PATTERN "Thirdparty" EXCLUDE)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/Thirdparty/
    DESTINATION include/ORB_SLAM2/Thirdparty
    COMPONENT library
FILES_MATCHING REGEX ".*\\.h(pp)?")

# ----------------------------------------------------------------------------
# Install package configuration file
# ----------------------------------------------------------------------------

# Install directory for CMake config files
set(ORB_SLAM2_CMAKE_INSTALL_DIR lib/cmake/${PROJECT_NAME})

# Configure the package config file from the template
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${PROJECT_SOURCE_DIR}/ORB_SLAM2Config.cmake.in # Input template
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake # Output file in build dir
    INSTALL_DESTINATION ${ORB_SLAM2_CMAKE_INSTALL_DIR}
)

# Install the generated package config file
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        DESTINATION ${ORB_SLAM2_CMAKE_INSTALL_DIR}
        COMPONENT library) 

# ----------------------------------------------------------------------------
# Python Bindings (Optional)
# ----------------------------------------------------------------------------
if(BUILD_PYTHON_BINDINGS)
    message(STATUS "Configuring Python bindings...")    
    add_subdirectory(python) 
else()
    message(STATUS "Skipping Python bindings configuration.")
endif()