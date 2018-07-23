
project(graph_slam)

################## VRO ############################
# execute_process(COMMAND rospack find visual_odometry OUTPUT_VARIABLE VO_ROOT)
# include_directories(/home/david/work/ros/indigo/src/visual_odometry/src/VRO)
include_directories("../../visual_odometry/src/VRO" ${CMAKE_CURRENT_SOURCE_DIR})

# message("STATUS VO_ROOT : ${VO_ROOT}")
# include_directories(${VO_ROOT}/src/VRO)

################### GTSAM 4.0 ######################
# set(GTSAM_ROOT /home/david/work/3rdlibs/gtsam-4.0/build)
set(GTSAM_ROOT /home/davidz/work/3rdlibs/gtsam-4.0/build)
 find_package(GTSAM REQUIRED 
   PATHS ${GTSAM_ROOT}) 
include_directories(${GTSAM_INCLUDE_DIR} ${GTSAM_ROOT}) # ${GTSAM_ROOT}/../gtsam/3rdparty/Eigen) # /usr/include/eigen3
link_directories(${GTSAM_ROOT}/gtsam )

#################### compile ######################
add_library(gtsam_graph gtsam_graph.cpp gt_parameter.cpp color.cpp)
target_link_libraries(gtsam_graph ${catkin_LIBRARIES} gtsam ${QT_LIBRARIES} vro ${OpenCV_LIBS} plane)

add_library(imu_interface imu_MEMS.cpp gyro_euler.cpp imu_base.cpp imu_vn100.cpp)
target_link_libraries(imu_interface ${catkin_LIBRARIES} gtsam)

################ gtsam graph slam ################
add_executable(test_gtsam_graph test_gt_graph.cpp)
target_link_libraries(test_gtsam_graph gtsam_graph vro sr4k_io)

add_executable(main_rgbdslam_offline main_rgbdslam_offline.cpp)
target_link_libraries(main_rgbdslam_offline gtsam_graph vro rs2h_io)

################ example in gtsam #############
# add_executable(imuFactorsExample ImuFactorsExample.cpp)
# target_link_libraries(imuFactorsExample gtsam_graph imu_interface ${catkin_LIBRARIES})

################ vro + imu_vn100 ############
add_executable(test_vro_imu_graph test_vro_imu_graph.cpp)
target_link_libraries(test_vro_imu_graph ${catkin_LIBRARIES} vro gtsam_graph imu_interface sr4k_io ${OpenCV_LIBS})

add_executable(main_rgbdslam_imu_offline main_rgbdslam_imu_offline.cpp)
target_link_libraries(main_rgbdslam_imu_offline ${catkin_LIBRARIES} gtsam_graph vro imu_interface rs2h_io ${OpenCV_LIBS})

############### ba + imu_vn100 #############
add_executable(test_ba_imu_graph test_ba_imu_graph.cpp)
target_link_libraries(test_ba_imu_graph ${catkin_LIBRARIES} vro gtsam_graph imu_interface sr4k_io ${OpenCV_LIBS})

############### hybrid + plane distance ###############
add_executable(test_plane_check_vo test_plane_check_vo.cpp)
target_link_libraries(test_plane_check_vo ${catkin_LIBRARIES} vro gtsam_graph imu_interface sr4k_io ${OpenCV_LIBS})

############### what parameters? ####################3
# add_executable(test_param test_param.cpp)
# target_link_libraries(test_param gtsam_graph ${catkin_LIBRARIES})


# add_subdirectory(test)
add_executable(test_imu_gravity test/test_imu_gravity.cpp)
target_link_libraries(test_imu_gravity ${catkin_LIBRARIES} gtsam_graph vro imu_interface rs2h_io ${OpenCV_LIBS})












