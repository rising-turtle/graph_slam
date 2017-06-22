
project(graph_slam)

################## VRO ############################
# execute_process(COMMAND rospack find visual_odometry OUTPUT_VARIABLE VO_ROOT)
# include_directories(/home/david/work/ros/indigo/src/visual_odometry/src/VRO)

include_directories("../../visual_odometry/src/VRO")

# message("STATUS VO_ROOT : ${VO_ROOT}")
# include_directories(${VO_ROOT}/src/VRO)

################### G2O ######################
# set(G2O_ROOT /home/david/work/3rdlibs/g2o/install)
set(G2O_ROOT /home/davidz/work/3rdlibs/g2o/install)
include_directories(${G2O_ROOT}/include ${G2O_ROOT}/include/EXTERNAL/csparse /usr/include/suitesparse) 
link_directories(${G2O_ROOT}/lib /usr/lib/i386-linux-gnu)
set(G2O_LIBS g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_core g2o_csparse_extension cxsparse)

#################### compile ######################
add_library(g2o_graph g2o_graph.cpp g2o_parameter.cpp color.cpp)
target_link_libraries(g2o_graph ${catkin_LIBRARIES} ${G2O_LIBS})

add_executable(test_g2o_graph test_g2o_graph.cpp)
target_link_libraries(test_g2o_graph g2o_graph vro sr4k_io)


