cmake_minimum_required(VERSION 3.8)
project(dz_minimal_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# changed to PickNik style reusable list -- will this be an issue with Eigen3/eigen_cmake_module
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  generate_parameter_library
)
# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#https://github.com/PickNikRobotics/generate_parameter_library#add-parameter-library-generation-to-project
set(GENERATE_PARAMETER_LIBRARY_NAME ${PROJECT_NAME}_generated_parameter_library)
message(STATUS "**** Generating Parameters for ${GENERATE_PARAMETER_LIBRARY_NAME} ****")
generate_parameter_library(
  ${GENERATE_PARAMETER_LIBRARY_NAME} # cmake target name for the parameter library
  src/${PROJECT_NAME}_generate_parameter_library_definition.yaml # path to input yaml file
)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> #maybe can't find params here? I don't think so, this is the same for pose tracker.
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "DZ_MINIMAL_CONTROLLER_BUILDING_LIBRARY")

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# -- for generate_parameter_library --
# Still a little unclear if this can't be done with ament_target_dependencies() but I'm not going to mess with it.
# https://github.com/PickNikRobotics/generate_parameter_library#add-parameter-library-generation-to-project
target_link_libraries(${PROJECT_NAME}
  ${GENERATE_PARAMETER_LIBRARY_NAME}
)

pluginlib_export_plugin_description_file(controller_interface dz_minimal_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
  #https://github.com/tylerjw/ros2_controllers/blob/2d8816c7d835f1957780ef61ee545495c8206d8a/admittance_controller/CMakeLists.txt#L57

install( 
  TARGETS ${PROJECT_NAME} ${GENERATE_PARAMETER_LIBRARY_NAME}
  EXPORT export_${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ======================================== TESTING DETAILS =============================================

if(BUILD_TESTING) #BUILD_TESTING
  #find_package(ament_lint_auto REQUIRED) # skipping linting for now

  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # gmock test for loading the controller -> this was from the ros2_control tutorial
  # ament_add_gmock(
  #   test_load_dz_minimal_controller # this is the name of the gmock test, not the same as the loaded controller
  #   test/test_load_dz_minimal_controller.cpp
  # )

  # instead we do this according to
  # https://github.com/PickNikRobotics/generate_parameter_library#use-example-yaml-files-in-tests
  add_rostest_with_parameters_gmock(
    test_load_dz_minimal_controller #the name of the test
    test/test_load_dz_minimal_controller.cpp #the name of the CPP file
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_parameters.yaml
  )
  #https://github.com/ros-controls/ros2_controllers/blob/eecb7f7af8cd750b9b4116947ae31fb5df52dcfd/admittance_controller/CMakeLists.txt
  target_include_directories(test_load_dz_minimal_controller PUBLIC ${GMOCK_INCLUDE_DIRS})
  target_link_libraries(test_load_dz_minimal_controller ${GMOCK_LIBRARIES} ${PROJECT_NAME} ${GENERATE_PARAMETER_LIBRARY_NAME}) #and project name?
  ament_target_dependencies(test_load_dz_minimal_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

# we SHOULD move to target-based installs ... works now
#ament_export_libraries(${PROJECT_NAME})

ament_export_targets(
  export_${PROJECT_NAME}Targets HAS_LIBRARY_TARGET
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
