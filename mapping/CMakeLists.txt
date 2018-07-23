

project(graph_slam)

# include_directories(/home/david/work/ros/indigo/src/visual_odometry/src/VRO)
include_directories("../../visual_odometry/src/VRO")

################### GTSAM 4.0 ######################
# set(GTSAM_ROOT /home/david/work/3rdlibs/gtsam-4.0/build)
set(GTSAM_ROOT /home/davidz/work/3rdlibs/gtsam-4.0/build)
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

add_executable(pcd2mesh pcd2mesh.cpp)
target_link_libraries(pcd2mesh ${PCL_LIBRARIES})

add_executable(pub_pcd pub_pcd.cpp)
target_link_libraries(pub_pcd ${catkin_LIBRARIES} ${PCL_LIBRARIES})

################## generate 3d map incrementally for recording ##############
add_executable(map_video map_video.cpp)
target_link_libraries(map_video gtsam_graph vro sr4k_io ${OpenCV_LIBS})

add_executable(map_video_mtraj map_video_mtraj.cpp)
target_link_libraries(map_video_mtraj gtsam_graph vro sr4k_io ${OpenCV_LIBS})

add_executable(map_video_rs map_video_rs.cpp)
target_link_libraries(map_video_rs gtsam_graph ${OpenCV_LIBS}  ${PCL_LIBRARIES} ${catkin_LIBRARIES} )




