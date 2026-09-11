/************************************************************************************
 *      Title       : param_handler.hpp
 *      Project     : MoveIt Pro ros2_force_controller integration
 *      Created     : 02/15/2024
 *      Author      : Emmanuel Akita
 *      Email       : efakita@utexas.edu
 *      Description : This provides a clean, organized way to grab parameters
 *                    from a yaml file for the MoveIt Studio version of the
 *                    force_controller.
 *      Copyright   : Copyright© The University of Texas at Austin, 2024. All
 *                    rights reserved.
 *
 *          All files within this directory are subject to the following, unless
 *          an alternative license is explicitly included within the text of
 *           each file.
 *
 *          This software and documentation constitute an unpublished work
 *          and contain valuable trade secrets and proprietary information
 *          belonging to the University. None of the foregoing material may be
 *          copied or duplicated or disclosed without the express, written
 *          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
 *          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
 *          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 *          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
 *          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
 *          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
 *          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
 *          University be liable for incidental, special, indirect, direct or
 *          consequential damages or loss of profits, interruption of business,
 *          or related expenses which may arise from use of software or
 *          documentation, including but not limited to those resulting from
 *          defects in software and/or documentation, or loss or inaccuracy of
 *          data of any kind.
 *
 **********************************************************************************/

#pragma once
#include <iostream>
#include <yaml-cpp/yaml.h>

#include "force_controller/utils/wrench_filters.hpp"

namespace twist_force_controller {

struct ForceControllerParams {
/**
 * @brief This struct holds the parameters for the force controller.
 * @details The parameters are set and stored in a struct for use in the force
 * controller object instantiation in the MoveIt Pro behavior. For a
 * comprehensive overview of each parameter, refer to the
 * force_controller_params.yaml file. Parameters relevant to the
 * one-dimensional force control scenario are made accessible via the behavior
 * UI for dynamic configuration during runtime. Other parameters are
 * designated as constants set by the behavior. To modify parameters, they can
 * be adjusted either through MoveIt Pro's UI or by directly editing the
 * corresponding C++ file and subsequently recompiling the package.
 */
double /*Behavior input port params: */ force_setpoint, kp, kd, ki, ee_weight,
    /*Controller config file params: */ ff_gain, max_twist, max_force,
    max_torque, anti_windup_error_threshold;

std::string
    /*Behavior input port params: */ ft_sensor_frame,
    gravity_frame, twist_pub_frame, dimension_str, pub_topic,
    /*Controller config file params: */ target_wrench_sub_topic,
    target_force_sub_topic, wrench_sub_topic;

std::vector<double>
    /*Behavior input port params: */ damping_1D, stiffness_1D, ee_com,
    /*Controller config file params: */ wrench_setpoint, stiffness_6D,
    damping_6D;

bool /*Behavior input port param: */ get_controller_twist,
    /*Controller config file params: */ has_ft_sensor, control_wrench,
    compensate_for_ee_weight, add_restoring_force_term, use_PID;

filter_utils::FilterWrenchParams filter_params;
};

/**
 * @brief Set parameters for the force controller object instantiation.

 * @details Default parameters for the data ports are provided in this
 * MoveIt Pro implementation of the yaml object fails to correctly load the
 * parameters. The remaining parameters are set as constants in the C++
 * implementation.
 * @post Params are loaded into class variables for use
 */
ForceControllerParams getControllerParams();

/**
 * @brief Get the default parameters for the force controller.
 *
 * @details This function is called when the config file is not found or
 * cannot be opened. It sets the default parameters for the force controller.
 * @post Default parameters are loaded into class variables for use
 */
ForceControllerParams getDefaultControllerParams();

} // namespace twist_force_controller



namespace hybrid_multidim_control {

  
/**
 * @brief Holds params related exclusively to the hybrid force motion controller.
 * 
 */
struct HybridMotionParams {
  /**
   * @brief This struct contains the configuration parameters related to the 
   * hybrid force motion controller. These are set in the same controller params
   * YAML file as the admittance force controller params. 
   * @details --- TO DO ---
   */
  double publish_time_length;
  std::string group_name;
};

/**
 * @brief Holds params input from action server goal (incoming from the action 
 * client behavior).
 * 
 */
struct HybridGoalParams {
  /**
   * @brief Struct to hold the params ported over from the action client, which
   * are set by input ports to the behavior tree. These inputs can be set manually
   * if the force controller is the only thing running, or could be inputs from 
   * other behaviors. 
   * 
   */
  geometry_msgs::msg::Wrench wrench_setpoint;
  double twist_val;
  double distance_delta;
  std::string dimension;    
};

/**
 * @brief Struct to hold ALL combined params derived from all 3 param structs.
 */

struct HybridForceExecParams 
  : public twist_force_controller::ForceControllerParams,
    public HybridMotionParams,
    public HybridGoalParams {
};

/**
 * @brief Get the Hybrid Motion Params object with params set in the cotroller 
 * params YAML file.
 * 
 * @return HybridMotionParams 
 */
HybridMotionParams getHybridMotionParams();

/**
 * @brief Get the default Hybrid Motion Params object if the controller YAML 
 * file is inaccessible or accessing any parameter in it fails. 
 * 
 * @return HybridMotionParams 
 */
HybridMotionParams getDefaultHybridParams();

} // namespace hybrid_multidim_control

