/************************************************************************************
 *      Title       : param_handler.hpp
 *      Project     : MoveIt Pro ros2_force_controller integration
 *      Created     : 02/15/2024
 *      Author      : Emmanuel Akita
 *      Email       : efakita@utexas.edu
 *      Description : This provides a clean, organized way to grab parameters
 *                    from a ymal file for the MoveIt Studio version of the
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
#include "force_controller/force_controller.hpp"
#include "force_controller/wrench_filters.hpp"

namespace twist_force_controller {

// /**
//  * @brief Set parameters for the force controller object instantiation.
//  *
//  *
//  * @details Default parameters for the data ports are provided in this
//  * MoveIt Pro implementation of the yaml object fails to correctly load the
//  * parameters. The remaining parameters are set as constants in the C++
//  * implementation.
//  * @post Params are loaded into class variables for use
//  */
// ForceControllerParams getControllerParams();

/**
 * @brief Get the default parameters for the force controller.
 *
 * @details This function is called when the config file is not found or
 * cannot be opened. It sets the default parameters for the force controller.
 * @post Default parameters are loaded into class variables for use
 */
ForceControllerParams getDefaultControllerParams();
} // namespace twist_force_controller
