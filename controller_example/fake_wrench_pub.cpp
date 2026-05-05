/************************************************************************************
 *      Title       : fake_wrench_pub.cpp
 *      Project     : force_controller
 *      Created     : 02/01/2024
 *      Author      : Emmanuel Akita
 *      Email       : efakita@utexas.edu
 *      Description : This is a WrenchPublisherNode class. It encapsulates a set of
 *                    utilities designed for publishing fake wrench data. 
 * 
 *      Copyright   : Copyright© The University of Texas at Austin, 2014-2023.
 *                    All rights reserved.
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

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class WrenchPublisherNode : public rclcpp::Node {
 public:
  WrenchPublisherNode(const rclcpp::NodeOptions& options)
      : Node("wrench_publisher", options) {
    // Initialize parameters from parameter server
    parseParameters();

    wrench_pub_ =
        create_publisher<geometry_msgs::msg::WrenchStamped>("/wrench_data", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&WrenchPublisherNode::publishWrench, this));
  }

 private:
  void parseParameters() {
    // Default values
    force_z_ = 2.0;
    max_threshold_ = 15.0;
    min_threshold_ = 5.0;

    // Load parameters from the parameter server
    force_z_ = this->declare_parameter<double>("force_z", force_z_);
    max_threshold_ =
        this->declare_parameter<double>("max_threshold", max_threshold_);
    min_threshold_ =
        this->declare_parameter<double>("min_threshold", min_threshold_);
  }

  void publishWrench() {
    // Generate some fake wrench data
    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = this->now();
    wrench_msg.wrench.force.z = force_z_;

    wrench_pub_->publish(wrench_msg);

    const double force_delta{0.5};
    // Update force_z_ for the next iteration
    if (increasing_) {
      force_z_ += force_delta;
      if (force_z_ >= max_threshold_) {
        increasing_ = false;
      }
    } else {
      force_z_ -= force_delta;
      if (force_z_ <= min_threshold_) {
        increasing_ = true;
      }
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool increasing_ = true;
  double force_z_;
  double max_threshold_;
  double min_threshold_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchPublisherNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
