cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)


SET(CMAKE_BUILD_TYPE Debug)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Cholmod REQUIRED)


find_package(Boost  1.54.0  	     # Minimum or EXACT version e.g. 1.36.0
  REQUIRED             					# Fail with error if Boost is not found
  COMPONENTS serialization 				# Boost libraries by their canonical name
  )


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/src
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${CHOLMOD_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
    src/inertial_ORB_SLAM/System.cc
    src/inertial_ORB_SLAM/Tracking.cc
    src/inertial_ORB_SLAM/LocalMapping.cc
    src/inertial_ORB_SLAM/LoopClosing.cc
    src/inertial_ORB_SLAM/ORBextractor.cc
    src/inertial_ORB_SLAM/ORBmatcher.cc
    src/inertial_ORB_SLAM/FrameDrawer.cc
    src/inertial_ORB_SLAM/Converter.cc
    src/inertial_ORB_SLAM/MapPoint.cc
    src/inertial_ORB_SLAM/KeyFrame.cc
    src/inertial_ORB_SLAM/Map.cc
    src/inertial_ORB_SLAM/MapDrawer.cc
    src/inertial_ORB_SLAM/Optimizer.cc
    src/inertial_ORB_SLAM/PnPsolver.cc
    src/inertial_ORB_SLAM/Frame.cc
    src/inertial_ORB_SLAM/KeyFrameDatabase.cc
    src/inertial_ORB_SLAM/Sim3Solver.cc
    src/inertial_ORB_SLAM/Initializer.cc
    src/inertial_ORB_SLAM/Viewer.cc


    src/IMU/configparam.h
    src/IMU/configparam.cpp

    src/IMU/imudata.h
    src/IMU/imudata.cpp
    src/IMU/IMUPreintegrator.h
    src/IMU/IMUPreintegrator.cpp
    src/IMU/so3.cpp
    src/IMU/so3.h
    src/IMU/NavState.h
    src/IMU/NavState.cpp

    src/IMU/g2otypes.h
    src/IMU/g2otypes.cpp
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
cholmod
${CHOLMOD_LIBRARIES}
${BLAS_LIBRARIES}
${LAPACK_LIBRARIES}
${Boost_LIBRARIES}
)


add_library(model_based_localization
        src/localization/model_based_localization.cpp
        )
target_link_libraries(model_based_localization
        ${PROJECT_NAME})

## Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/VIO)
add_executable(euroc_VIO
Examples/VIO/euroc_VIO.cc)
target_link_libraries(euroc_VIO ${PROJECT_NAME})

add_executable(segway_VIO
        Examples/VIO/segway_VIO.cc
        Examples/DataSetUtils/segwayDatasetReader.hpp
        Examples/DataSetUtils/segwayDatasetReader.cpp)
target_link_libraries(segway_VIO ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

#add_executable(rgbd_tum
#Examples/RGB-D/rgbd_tum.cc)
#target_link_libraries(rgbd_tum ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

#add_executable(stereo_kitti
#Examples/Stereo/stereo_kitti.cc)
#target_link_libraries(stereo_kitti ${PROJECT_NAME})

#add_executable(stereo_euroc
#Examples/Stereo/stereo_euroc.cc)
#target_link_libraries(stereo_euroc ${PROJECT_NAME})


#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

#add_executable(mono_tum
#Examples/Monocular/mono_tum.cc)
#target_link_libraries(mono_tum ${PROJECT_NAME})

#add_executable(mono_kitti
#Examples/Monocular/mono_kitti.cc)
#target_link_libraries(mono_kitti ${PROJECT_NAME})

#add_executable(mono_euroc
#Examples/Monocular/mono_euroc.cc)
#target_link_libraries(mono_euroc ${PROJECT_NAME})

## Build tools
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
#add_executable(bin_vocabulary
#tools/bin_vocabulary.cc)
#target_link_libraries(bin_vocabulary ${PROJECT_NAME})


## Test

# Prepare "Catch" library for other executables
set(CATCH_INCLUDE_DIR include/utility/catch)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR} )

add_subdirectory(test)
