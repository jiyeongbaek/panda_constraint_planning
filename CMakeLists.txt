cmake_minimum_required (VERSION 2.6)

project (constraint_planner)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_simple_controller_manager
  moveit_planners_ompl
  moveit_visual_tools
  moveit_resources
  trajectory_msgs
)

LIST( APPEND CMAKE_MODULE_PATH "$ENV{HOME}/cmake" )
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED system)
find_package(ompl REQUIRED)
find_package(fcl REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    moveit_core 
    #moveit_planners_ompl
    moveit_ros_planning_interface
    moveit_simple_controller_manager
    rviz_visual_tools
    
  LIBRARIES 
    ${PROJECT_NAME} 
  INCLUDE_DIRS  #which is neede to indicate the include path
    include
    ${catkin_INCLUDE_DIRS}
    # "/home/jiyeong/catkin_ws/devel/include" 
    ${OMPL_INCLUDE_DIRS}
  DEPENDS
    EIGEN3
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}    
    /usr/include/eigen3 
    /usr/inclue/boost 
    /usr/local/include/ompl
    ${RBDL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${moveit_resources_INCLUDE_DIRS}
)

link_directories(${Boost_LIBRARY_DIR})

set(SOURCES
  src/base/jy_ProjectedStateSpace.cpp
  src/planner/newPRM.cpp
  src/planner/No_RRT.cpp
  src/planner/NoRandomSampleSpace.cpp
  src/panda_model_updater.cpp)

add_library(${PROJECT_NAME}_lib
  ${SOURCES}
)

add_executable(test src/main.cpp)

target_link_libraries (test              
                      ${catkin_LIBRARIES}          
                       ${OMPL_LIBRARIES}
                       ${Boost_LIBRARIES}
                       rbdl
                       fcl
                       ${PROJECT_NAME}_lib)


add_executable(path_smoothing_test src/trajectory_smoothing.cpp)

target_link_libraries (path_smoothing_test              
                      ${catkin_LIBRARIES}          
                       ${OMPL_LIBRARIES}
                       ${Boost_LIBRARIES}
                       rbdl
                       fcl
                       ${PROJECT_NAME}_lib)


