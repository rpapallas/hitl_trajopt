cmake_minimum_required (VERSION 2.8.3)
project(or_hitl)

set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(compilerSettings)
include(boost)
include(includes)
include(mujoco)
include(openmp)

include_directories(${Boost_INCLUDE_DIRS} 
                    ${OMPL_INCLUDE_DIRS} 
                    ${catkin_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS} 
                    ${Mujoco_INCLUDE_DIRS}
                    "include")

link_directories(${Boost_LIBRARY_DIRS} 
                 ${OMPL_LIBRARY_DIRS}
                 ${CMAKE_CURRENT_SOURCE_DIR}/lib/mjpro200/)

include(trajectoryOptimizationBasedPlanning)
