# CMakeLists.txt for ORB_SLAM2 Python Bindings (as a subdirectory)

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

find_package(NumPy REQUIRED)
find_package(PythonLibs 3.10 REQUIRED)
find_package(Boost 1.80.0 REQUIRED COMPONENTS python310)

# RPATH fiddling for the library, so that the installed version can find the linked libraries
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(TARGET_MODULE_NAME orbslam2)
add_library(${TARGET_MODULE_NAME} SHARED
src/ORBSlamPython.cpp
src/pyboost_cv4_converter.cpp)

target_include_directories(${TARGET_MODULE_NAME} PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/include
${Boost_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${PYTHON_INCLUDE_DIRS}
${PYTHON_NUMPY_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS} 
)
set_target_properties(${TARGET_MODULE_NAME} PROPERTIES PREFIX "")

target_link_libraries(${TARGET_MODULE_NAME} 
${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Boost_LIBRARIES}
${PYTHON_LIBRARIES}
${Pangolin_LIBRARIES}
)

install(TARGETS ${TARGET_MODULE_NAME} 
        DESTINATION lib/python3.10/site-packages)
        
