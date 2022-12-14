cmake_minimum_required(VERSION 3.8)
project(dz_minimal_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# changed to PickNik style reusable list -- will this be an issue with Eigen3/eigen_cmake_module
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs  
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(dz_minimal_controller SHARED src/dz_minimal_controller.cpp)
target_include_directories(dz_minimal_controller PRIVATE include)
generate_parameter_library(dz_minimal_controller_generated_parameter_library src/dz_minimal_controller_generate_parameter_library_definition.yaml)
target_link_libraries(dz_minimal_controller dz_minimal_controller_generated_parameter_library)
ament_target_dependencies(dz_minimal_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(dz_minimal_controller PRIVATE "DZ_MINIMAL_CONTROLLER_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(controller_interface dz_minimal_controller.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)
  #https://github.com/tylerjw/ros2_controllers/blob/2d8816c7d835f1957780ef61ee545495c8206d8a/admittance_controller/CMakeLists.txt#L57

install(TARGETS dz_minimal_controller dz_minimal_controller_generated_parameter_library
  EXPORT export_dz_minimal_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(control_msgs REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(controller_interface REQUIRED) #why find if I don't use? GMock?
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

# test loading dz_minimal controller
  add_rostest_with_parameters_gmock(test_load_dz_minimal_controller test/test_load_dz_minimal_controller.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_parameters.yaml)
  target_include_directories(test_load_dz_minimal_controller PUBLIC ${GMOCK_INCLUDE_DIRS})
  target_link_libraries(test_load_dz_minimal_controller ${GMOCK_LIBRARIES})
  ament_target_dependencies(
    test_load_dz_minimal_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
endif()

ament_export_targets(
  export_dz_minimal_controller HAS_LIBRARY_TARGET
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_package()
