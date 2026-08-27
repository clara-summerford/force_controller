#pragma once
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace hybrid_multidim_utils {
/**
 * @brief Adds two twist messages.
 * @param twist1 The first twist message.
 * @param twist2 The second twist message.
 * @return geometry_msgs::msg::Twist The sum of the two twist messages.
 */
geometry_msgs::msg::Twist addTwists(const geometry_msgs::msg::Twist &twist1,
                                    const geometry_msgs::msg::Twist &twist2);

/**
 * @brief Converts a 6x1 vector to wrench msg.
 * @param wrench Desired wrench.
 * @param value input vector to transform to wrench msg.
 */
void toWrenchMsg(geometry_msgs::msg::WrenchStamped& wrench,
                    const std::vector<double>& value);

/**
 * @brief   Rotates the linear twist message about the specified axis by the
 *          specified angle.
 * @param   linear_twist The linear twist message to rotate.
 * @param   axis The axis to rotate around. Must be 'x', 'y', or 'z'.
 * @param   theta_degrees The angle to rotate by in degrees.
 */
void rotateLinearTwist(geometry_msgs::msg::Twist &linear_twist, char axis,
                       double theta_degrees);

/**
 * @brief Computes the Euclidean distance moved by the end effector from the
 * initial pose to the current pose.
 * @note This is a simple implementation that assumes the end effector moves
 * in a straight line. It does not account non-linear paths and
 * workspace constraints.
 * @param current_pose The current pose of the end effector.
 * @return double distance moved by the end effector.
 */
double calculateDistanceMoved(const geometry_msgs::msg::Pose &current_pose,
                              const geometry_msgs::msg::Pose &initial_pose);

/**
 * @brief Computes the angle turned by the end effector from the initial pose
 * to the current pose.
 * @note This does not account non-linear paths and workspace constraints.
 * @param current_pose The current pose of the end effector.
 * @return double angle turned by the end effector in radians.
 */
double calculateAngleTurned(const geometry_msgs::msg::Pose &current_pose,
                            const geometry_msgs::msg::Pose &initial_pose);




} // namespace hybrid_multidim_utils