#include "glovebox_bt/utils/param_handler.hpp"
#include <cstdlib> // for std::getenv
#include <fstream> //
#include <yaml-cpp/yaml.h>

namespace twist_force_controller {
ForceControllerParams param;

// ForceControllerParams getControllerParams() {

//   /* Set the force controller values.
//    Review the force_controller_params.yaml file for more details on each
//    parameter
//   */

//   // Grab the path to the config file
//   std::string configFilePath = std::getenv("CONFIG_FILE_PATH");
//   if (configFilePath.empty()) {
//     std::cerr << "Error: CONFIG_FILE_PATH environment variable is not set or "
//                  "empty.  Using default param values.\n"
//               << std::endl;
//     return getDefaultControllerParams();
//   }

//   // Open the YAML config file
//   std::ifstream configFile(configFilePath);
//   if (!configFile.is_open()) {
//     std::cerr
//         << "ERROR: Unable to open config file. Using default param values.\n";
//     return getDefaultControllerParams();
//   }

//   // Load YAML file contents
//   YAML::Node force_controller_params = YAML::LoadFile(configFilePath);

//   // Access the YAML data
//   param.target_force_sub_topic =
//       force_controller_params["target_force_sub_topic"].as<std::string>();
//   param.control_wrench = force_controller_params["control_wrench"].as<bool>();
//   param.target_wrench_sub_topic =
//       force_controller_params["target_wrench_sub_topic"].as<std::string>();
//   param.has_ft_sensor = force_controller_params["has_ft_sensor"].as<bool>();
//   param.ff_gain = force_controller_params["ff_gain"].as<double>();
//   param.anti_windup_error_threshold =
//       force_controller_params["anti_windup_error_threshold"].as<double>();
//   param.compensate_for_ee_weight =
//       force_controller_params["compensate_for_ee_weight"].as<bool>();
//   param.max_torque = force_controller_params["max_torque"].as<double>();
//   param.filter_params.deadband_threshold =
//       force_controller_params["deadband_threshold"].as<double>();
//   param.filter_params.apply_ema_filter =
//       force_controller_params["apply_ema_filter"].as<bool>();
//   param.filter_params.alpha = force_controller_params["ema_alpha"].as<double>();
//   param.filter_params.apply_bw_filter =
//       force_controller_params["apply_bw_filter"].as<bool>();
//   param.filter_params.cut_off_frequency =
//       force_controller_params["cutoff_freq"].as<double>();
//   param.filter_params.dt =
//       force_controller_params["sampling_time"].as<double>();
//   param.filter_params.apply_kf_filter =
//       force_controller_params["apply_kf_filter"].as<bool>();
//   param.filter_params.process_noise =
//       force_controller_params["process_noise"].as<double>();
//   param.filter_params.measurement_noise =
//       force_controller_params["measurement_noise"].as<double>();

//   // Handle the vector cases
//   if (force_controller_params["wrench_setpoint"] &&
//       force_controller_params["wrench_setpoint"].IsSequence()) {
//     param.wrench_setpoint =
//         force_controller_params["wrench_setpoint"].as<std::vector<double>>();
//   } else {
//     param.wrench_setpoint = {0, 0, 0, 0, 0, 0};
//   }
//   if (force_controller_params["damping_6D"] &&
//       force_controller_params["damping_6D"].IsSequence()) {
//     param.damping_6D =
//         force_controller_params["damping_6D"].as<std::vector<double>>();
//   } else {
//     param.damping_6D = {1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0};
//   }
//   if (force_controller_params["stiffness_6D"] &&
//       force_controller_params["stiffness_6D"].IsSequence()) {
//     param.stiffness_6D =
//         force_controller_params["stiffness_6D"].as<std::vector<double>>();
//   } else {
//     param.stiffness_6D = {2000.0, 2000.0, 2000.0, 100.0, 100.0, 100.0};
//   }
//   if (force_controller_params["ee_com"] &&
//       force_controller_params["ee_com"].IsSequence()) {
//     param.ee_com = force_controller_params["ee_com"].as<std::vector<double>>();
//   } else {
//     param.ee_com = {0.0, 0.0, 0.06};
//   }

//   return param;
// }

ForceControllerParams getDefaultControllerParams() {
  /* Set default force controller parameter values.
   Review the force_controller_params.yaml file for more details on each
   parameter
  */
  ForceControllerParams param;
  param.has_ft_sensor = true;
  param.add_restoring_force_term = false;
  param.get_controller_twist = true;
  param.control_wrench = false;
  param.compensate_for_ee_weight = false;
  param.use_PID = true;

  param.force_setpoint = 0.0;
  param.max_force = 20.0;
  param.max_twist = 0.5;
  param.ee_weight = 9.02;
  param.ff_gain = 0.0;
  param.max_torque = 10.0;
  param.anti_windup_error_threshold = 10.0;

  param.dimension_str = "X";
  param.ft_sensor_frame = "grasp_link";
  param.gravity_frame = "base_link";
  param.twist_pub_frame = "grasp_link";
  param.target_force_sub_topic = "target_force_data";
  param.target_wrench_sub_topic = "target_wrench_data";
  param.wrench_sub_topic = "/force_torque_sensor_broadcaster/wrench";
  param.pub_topic = "/servo_server/delta_twist_cmds";

  param.damping_1D = {1000.0};
  param.stiffness_1D = {2000.0};
  param.ee_com = {0.0, 0.0, 0.06};
  param.wrench_setpoint = {0, 0, 0, 0, 0, 0};
  param.damping_6D = {1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0};
  param.stiffness_6D = {2000.0, 2000.0, 2000.0, 100.0, 100.0, 100.0};

  param.filter_params.filter_order = {filter_utils::FilterType::EMA,
                                      filter_utils::FilterType::Kalman};
  param.filter_params.deadband_threshold = 0.0;
  param.filter_params.apply_ema_filter = true;
  param.filter_params.alpha = 0.8;
  param.filter_params.apply_bw_filter = false;
  param.filter_params.cut_off_frequency = 100.0;
  param.filter_params.dt = 0.001;
  param.filter_params.apply_kf_filter = true;
  param.filter_params.process_noise = 0.07;
  param.filter_params.measurement_noise = 0.01;
  return param;
}
} // namespace twist_force_controller
