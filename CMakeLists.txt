cmake_minimum_required(VERSION 2.8)
project(camera_lidar_calibration)

set(CMAKE_CXX_STANDARD 11)


find_package(OpenCV 3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/config.cc
        )

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${CERES_LIBRARIES}
        )

add_executable(camera_laser_calibration src/camera_lidar_calibration.cc)
target_link_libraries(camera_laser_calibration ${PROJECT_NAME})

add_executable(corner_detecter src/corner_detecter.cc)
target_link_libraries(corner_detecter ${PROJECT_NAME})

add_executable(reprojection src/reprojection.cc)
target_link_libraries(reprojection ${PROJECT_NAME})
