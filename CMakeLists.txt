cmake_minimum_required(VERSION 2.8)
project(camera_lidar_calibration)

set(CMAKE_CXX_STANDARD 11)

## Test modele, if you do not have aruco, just comment these lines

set(ARUCO_PATH /usr/local)
SET(CMAKE_MODULE_PATH ${ARUCO_PATH}/lib/cmake)
SET(ARUCO_INCLUDE_DIRS ${ARUCO_PATH}/include/aruco )

find_package(aruco REQUIRED )
## Test modele, if you do not have aruco, just comment these lines
#SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
MESSAGE(${ARUCO_INCLUDE_DIRS} )
## Test modele, if you do not have aruco, just comment these lines

find_package(OpenCV 3.3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
## Test modele, if you do not have aruco, just comment these lines

## Test modele, if you do not have aruco, just comment these lines

INCLUDE(FindPkgConfig)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${ARUCO_INCLUDE_DIRS} ## Test modele, if you do not have aruco, just comment these lines
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/config.cc
        )

LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/res)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${CERES_LIBRARIES}
        ${aruco_LIBS}
#        ${ARUCO_LIBS}
        )

add_executable(camera_laser_calibration src/camera_lidar_calibration.cc)
target_link_libraries(camera_laser_calibration ${PROJECT_NAME})

add_executable(corner_detecter src/corner_detecter.cc)
target_link_libraries(corner_detecter ${PROJECT_NAME})

add_executable(reprojection src/reprojection.cc)
target_link_libraries(reprojection ${PROJECT_NAME})

add_executable(detect_plane_with_aruco src/detect_plane_with_aruco.cc)
target_link_libraries(detect_plane_with_aruco ${PROJECT_NAME}
#        ${OpenCV_LIBS}
#        ${ARUCO_LIBRARIES}
#        ${aruco_LIBS}
        )
