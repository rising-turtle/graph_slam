

project(graph_slam)

include_directories(/home/david/work/ros/indigo/src/visual_odometry/src/VRO)

################### GTSAM 4.0 ######################
set(GTSAM_ROOT /home/david/work/3rdlibs/gtsam-4.0/build)
 find_package(GTSAM REQUIRED 
   PATHS ${GTSAM_ROOT}) 
include_directories(${GTSAM_INCLUDE_DIR} ${GTSAM_ROOT}) # ${GTSAM_ROOT}/../gtsam/3rdparty/Eigen) # /usr/include/eigen3
link_directories(${GTSAM_ROOT}/gtsam )


################## compile #########################
add_executable(mapping_PLY mapping_PLY.cpp)
target_link_libraries(mapping_PLY gtsam vro sr4k_io ${OpenCV_LIBS})


add_executable(mapping_PCD mapping_PCD.cpp)
target_link_libraries(mapping_PCD gtsam_graph vro sr4k_io)

add_executable(mapping_PLY_rs mapping_PLY_rs.cpp)
target_link_libraries(mapping_PLY_rs gtsam vro sr4k_io ${OpenCV_LIBS})

 add_executable(mapping_PCD_rs mapping_PCD_rs.cpp)
 target_link_libraries(mapping_PCD_rs gtsam_graph vro sr4k_io ${OpenCV_LIBS})

