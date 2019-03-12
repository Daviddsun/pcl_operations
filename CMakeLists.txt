cmake_minimum_required(VERSION 3.12)
project(super_voxel)

set(CMAKE_CXX_STANDARD 14)

# 增加PCL库的依赖
FIND_PACKAGE( PCL 1.7 REQUIRED COMPONENTS common io visualization segmentation people sample_consensus)

# 添加头文件和库文件
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )

# octomap
FIND_PACKAGE( octomap REQUIRED )
INCLUDE_DIRECTORIES( ${OCTOMAP_INCLUDE_DIRS} )


FIND_PACKAGE(OpenCV REQUIRED)
find_package(Eigen3)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
)


add_executable(super_voxel main.cpp)
TARGET_LINK_LIBRARIES( super_voxel
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES})

add_executable(octo_tree octo_tree.cpp)
TARGET_LINK_LIBRARIES( octo_tree
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES})

add_executable(for_background for_background.cpp)
TARGET_LINK_LIBRARIES( for_background
        ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
        ${EIGEN3_INCLUDE_DIR}
        )
#
#add_executable(people people.cpp)
#TARGET_LINK_LIBRARIES( people
#        ${PCL_LIBRARIES}
#        )

add_executable(projection projection.cpp)
TARGET_LINK_LIBRARIES( projection
        ${PCL_LIBRARIES}
        )