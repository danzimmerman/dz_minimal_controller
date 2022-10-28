## Dz Minimal Controller

This is a debugging effort for a shareable minimal reproduction of a ROS2 Control controller using
 * PickNik's `generate_parameter_library`
 * GMock testing

I'm following along with `admittance_controller` and `joint_trajectory_controller` but didn't want
to copy either.

The full error I get initially is

```log
(humble) user@computer:~/ros/minimal_ws$ colcon test-result --verbose
build/dz_minimal_controller/Testing/20221028-2257/Test.xml: 1 test, 0 errors, 1 failure, 0 skipped
- test_load_dz_minimal_controller
  <<< failure message
    -- run_test.py: invoking following command in '/home/dan/ros/minimal_ws/build/dz_minimal_controller':
     - /home/dan/ros/minimal_ws/build/dz_minimal_controller/test_load_dz_minimal_controller --ros-args --params-file /home/dan/ros/minimal_ws/src/dz_minimal_controller/test/test_parameters.yaml -- --gtest_output=xml:/home/dan/ros/minimal_ws/build/dz_minimal_controller/test_results/dz_minimal_controller/test_load_dz_minimal_controller.gtest.xml
    [==========] Running 1 test from 1 test suite.
    [----------] Global test environment set-up.
    [----------] 1 test from TestLoadDzMinimalController
    [ RUN      ] TestLoadDzMinimalController.load_controller
    [INFO] [1666997839.305985731] [resource_manager]: Loading hardware 'TestActuatorHardware'
    unknown file: Failure
    C++ exception with description "According to the loaded plugin descriptions the class test_actuator with base class type hardware_interface::ActuatorInterface does not exist. Declared types are  robotiq_driver/RobotiqGripperHardwareInterface" thrown in the test body.
    [  FAILED  ] TestLoadDzMinimalController.load_controller (1 ms)
    [----------] 1 test from TestLoadDzMinimalController (1 ms total)

    [----------] Global test environment tear-down
    [==========] 1 test from 1 test suite ran. (2 ms total)
    [  PASSED  ] 0 tests.
    [  FAILED  ] 1 test, listed below:
    [  FAILED  ] TestLoadDzMinimalController.load_controller

     1 FAILED TEST
    -- run_test.py: return code 1
    -- run_test.py: inject classname prefix into gtest result file '/home/dan/ros/minimal_ws/build/dz_minimal_controller/test_results/dz_minimal_controller/test_load_dz_minimal_controller.gtest.xml'
    -- run_test.py: verify result file '/home/dan/ros/minimal_ws/build/dz_minimal_controller/test_results/dz_minimal_controller/test_load_dz_minimal_controller.gtest.xml'
  >>>
build/dz_minimal_controller/test_results/dz_minimal_controller/test_load_dz_minimal_controller.gtest.xml: 1 test, 0 errors, 1 failure, 0 skipped
- dz_minimal_controller.TestLoadDzMinimalController load_controller
  <<< failure message
    unknown file
    C++ exception with description "According to the loaded plugin descriptions the class test_actuator with base class type hardware_interface::ActuatorInterface does not exist. Declared types are  robotiq_driver/RobotiqGripperHardwareInterface" thrown in the test body.
  >>>

Summary: 2 tests, 0 errors, 2 failures, 0 skipped
```