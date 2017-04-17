
project(graph_slam)

include_directories("../")

################ example in gtsam #############
# add_executable(imuFactorsExample ImuFactorsExample.cpp)
# target_link_libraries(imuFactorsExample gtsam_graph imu_interface ${catkin_LIBRARIES})


############## test bundle adjustment ########## 
add_executable(test_ba test_ba.cpp)
target_link_libraries(test_ba ${catkin_LIBRARIES} gtsam_graph sr4k_io vro )

add_executable(vo2ba convert_vo2ba.cpp)
target_link_libraries(vo2ba ${catkin_LIBRARIES} gtsam_graph sr4k_io vro )

add_executable(vo2ba_new convert_vo2ba_2.cpp)
target_link_libraries(vo2ba_new ${catkin_LIBRARIES} gtsam_graph sr4k_io vro )

############## test plane distance #########
add_executable(test_plane_dis test_plane_dis.cpp)
target_link_libraries(test_plane_dis ${catkin_LIBRARIES} gtsam_graph sr4k_io vro )

add_executable(patch_plane_dis test_patch_plane_dis.cpp)
target_link_libraries(patch_plane_dis ${catkin_LIBRARIES} gtsam_graph sr4k_io vro )

############## refine vro results ############
add_executable(refine_vo delete_vo_by_plane_check.cpp)
target_link_libraries(refine_vo ${catkin_LIBRARIES} gtsam_graph sr4k_io vro )

############## add vro results to sparse area ############ 
add_executable(add_vo_at_sparse_area add_vo_at_sparse_area.cpp)
target_link_libraries(add_vo_at_sparse_area ${catkin_LIBRARIES} gtsam_graph sr4k_io vro )

add_executable(view_registration view_registration_results.cpp)
target_link_libraries(view_registration ${catkin_LIBRARIES} gtsam_graph sr4k_io vro )

add_executable(view_registration_list view_registration_list.cpp)
target_link_libraries(view_registration_list ${catkin_LIBRARIES} gtsam_graph sr4k_io vro )

add_executable(view_registration_range view_registration_range.cpp)
target_link_libraries(view_registration_range ${catkin_LIBRARIES} gtsam_graph sr4k_io vro )

add_executable(compute_trans_diff compute_trans_difference.cpp)
target_link_libraries(compute_trans_diff ${catkin_LIBRARIES} gtsam_graph sr4k_io vro )

add_executable(merge_vo merge_vro.cpp)
target_link_libraries(merge_vo ${catkin_LIBRARIES} gtsam_graph sr4k_io vro )

############### what param ####################3
# add_executable(test_param test_param.cpp)
# target_link_libraries(test_param gtsam_graph ${catkin_LIBRARIES})

# add_executable(testOrientedPlane testOrientedPlane3.cpp)
# target_link_libraries(testOrientedPlane gtsam_graph CppUnitLite)

# add_executable(testOrientedPlaneFactor testOrientedPlane3Factor.cpp)
# target_link_libraries(testOrientedPlaneFactor gtsam_graph CppUnitLite)

############### display sr4k img and IMU_MEMS measurement ##############
# add_executable(test_imu_MEMS test_imu_MEMS.cpp)
# target_link_libraries(test_imu_MEMS sr4k_io imu_interface ${OpenCV_LIBS})

############### test vro + imu_MEMS ################
# add_executable(test_gt_imu_MEMS test_gt_imu_MEMS.cpp)
# target_link_libraries(test_gt_imu_MEMS sr4k_io vro imu_interface ${OpenCV_LIBS} ${catkin_LIBRARIES} gtsam_graph gtsam)

############### display sr4k img and IMU_VN100 measurement #####################
 add_executable(test_vn100 test_imu_vn100.cpp)
 target_link_libraries(test_vn100 gtsam_graph vro sr4k_io imu_interface ${OpenCV_LIBS} ${catkin_LIBRARIES})

############### vro match two sr4k frames #############################
 add_executable(test_match_two_frames test_match_two_frames.cpp)
 target_link_libraries(test_match_two_frames ${catkin_LIBRARIES} gtsam_graph vro sr4k_io)

 add_executable(test_match_pnp test_match_PnP.cpp)
 target_link_libraries(test_match_pnp ${catkin_LIBRARIES} gtsam_graph vro sr4k_io)

################# plane propagation ####################
 add_executable(test_plane_propagate test_plane_propagate.cpp display_many_imgs.cpp)
 target_link_libraries(test_plane_propagate ${catkin_LIBRARIES} gtsam_graph sr4k_io ${OpenCV_LIBS})

##
 add_executable(test_bug test_no_idea.cpp)
 target_link_libraries(test_bug ${catkin_LIBRARIES} gtsam_graph sr4k_io ${OpenCV_LIBS})

 add_executable(test_quaternion testQuaternion.cpp)
 target_link_libraries(test_quaternion ${catkin_LIBRARIES} gtsam_graph)












