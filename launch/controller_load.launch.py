import launch
import launch_ros
#this is PickNik"s launch_param_builder https://github.com/PickNikRobotics/launch_param_builder
import launch_param_builder 

def launch_setup(context, *args, **kwargs):

  robot_description = launch_param_builder.ParameterBuilder("ur_description").xacro_parameter(
    parameter_name="robot_description",
    file_path="urdf/ur.urdf.xacro", 
    mappings={ #use a fake UR5e robot
      "name":"ur", 
      "ur_type":"ur5e",
      "use_fake_hardware":"true"
    } 
  ).to_dict()

  control_file_params = (
    launch_param_builder.ParameterBuilder("dz_minimal_controller")
    .path_parameter("update_rate_file", "config/update_rate.yaml")
    .path_parameter("controllers_file", "config/controller_params.yaml")
  ).to_dict()
  
  controller_manager_node = launch_ros.actions.Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
      robot_description, 
      control_file_params["update_rate_file"], 
      control_file_params["controllers_file"]],
    output="screen",
  )

  dz_spawner_node = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    arguments=["dz_controller", "--controller-manager", "/controller_manager"]
  )

  return [controller_manager_node, dz_spawner_node]

def generate_launch_description():
  return launch.LaunchDescription([launch.actions.OpaqueFunction(function=launch_setup)])
