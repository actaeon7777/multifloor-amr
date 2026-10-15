cmake_minimum_required(VERSION 3.16)
project(amr_tim_localization LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
 set(CMAKE_BUILD_TYPE Release)
endif()
find_package(ament_cmake REQUIRED)
foreach(dep rclcpp rcl_interfaces nav_msgs sensor_msgs geometry_msgs amr_localization_interfaces tf2 tf2_geometry_msgs tf2_eigen angles)
 find_package(${dep} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree filters registration)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(octomap REQUIRED)
find_package(OpenMP REQUIRED)
set(CORE_DIR src/relocalization/utils)
add_library(tim_core
 ${CORE_DIR}/math_utils.cpp
 ${CORE_DIR}/relocalization_2d/likelihood_field_2d.cpp
 ${CORE_DIR}/relocalization_2d/sector_query_2d.cpp
 ${CORE_DIR}/relocalization_2d/area_set_2d.cpp
 ${CORE_DIR}/relocalization_2d/gicp_2d.cpp
 ${CORE_DIR}/relocalization_2d/sampling_2d.cpp
 ${CORE_DIR}/relocalization_2d/relocalization_2d.cpp
 src/localizer.cpp
 src/fine_refinement.cpp)
target_include_directories(tim_core PUBLIC include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(tim_core rclcpp nav_msgs sensor_msgs geometry_msgs amr_localization_interfaces tf2 tf2_geometry_msgs tf2_eigen angles)
target_link_libraries(tim_core ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OCTOMAP_LIBRARIES} OpenMP::OpenMP_CXX Eigen3::Eigen)
# No -ffast-math: invalid and infinite LaserScan samples must retain IEEE semantics.
add_executable(tim_global_localizer src/tim_global_localizer.cpp)
ament_target_dependencies(tim_global_localizer rcl_interfaces)
target_link_libraries(tim_global_localizer tim_core)
install(TARGETS tim_core tim_global_localizer ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE UPSTREAM.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
 find_package(ament_cmake_gtest REQUIRED)
 ament_add_gtest(test_tim_core test/test_tim_core.cpp)
 target_link_libraries(test_tim_core tim_core)
endif()
ament_package()
