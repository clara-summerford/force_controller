/************************************************************************************
 *      Title       : wrench_filters.hpp
 *      Project     : force_controller
 *      Created     : 01/17/2024
 *      Author      : Emmanuel Akita
 *      Email       : efakita@utexas.edu
 *      Description : This is a FilterUtils class. It encapsulates a set of
 *                    filtering utilities designed for signal processing
 *                    applications. It provides a modular and organized
 *                    framework for implementing Exponential Moving Average
 *                    (EMA), Butterworth, and Kalman filters. The class
 *                    maintains internal state variables for each filter type,
 *                    allowing for independent instances with distinct
 *                    configurations.
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

#pragma once

#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace filter_utils {
/*
Force - torque sensor data is typically noisy due to a variety of factors, such
as sensor vibrations, electrical interference, and mechanical vibrations. Using
a filter can help to remove this noise and provide a smoother signal that is
easier to control.
*/
/**
 * @brief The type of filter to be applied
 */
enum class FilterType { EMA, Butterworth, Kalman };

/**
 * @brief Function to convert a filter type string to its corresponding
 * FilterType enum.
 * @details This function checks if the given filter type string is present in a
 * predefined map that associates filter type strings with
 * their corresponding enum values. If found, it returns the associated
 * FilterType enum; otherwise, it returns a default value, FilterType::EMA.
 *
 * @param str The input filter type string to be converted.
 * @return The corresponding FilterType enum for the given string, or
 * FilterType::EMA as a default.
 */
FilterType stringToFilterType(const std::string& str);

/**
 * @brief This struct holds parameters for filtering wrench data using
 * various filters.
 * @details This struct defines parameters that control the behavior of wrench
 * data filtering. It includes options for applying Exponential Moving Average
 * (EMA), Butterworth, and Kalman filters, as well as settings for deadband,
 * smoothing, and noise levels.
 */
struct FilterWrenchParams {
  /** @param apply_ema Whether to apply the Exponential Moving Average (EMA)
   * filter
   * @param apply_bw Whether to apply the Butterworth filter
   * @param apply_kf Whether to apply the Kalman filter
   * @param deadband_threshold The deadband threshold value
   * @param alpha The smoothing factor for the EMA filter
   * @param cut_off_frequency The cutoff frequency for the Butterworth filter
   * @param dt The sampling time
   * @param process_noise The process noise for the Kalman filter
   * @param measurement_noise The measurement noise for the Kalman filter
   * @param filter_order The order of the filters to be applied
   */
  bool apply_ema_filter;
  bool apply_bw_filter;
  bool apply_kf_filter;
  double deadband_threshold;
  double alpha;
  double cut_off_frequency;
  double dt;
  double process_noise;
  double measurement_noise;
  std::vector<FilterType> filter_order;
};

class FilterUtils {
 public:
  FilterUtils();
  ~FilterUtils();

  /**
   * @brief Applies a deadband to the input sensor data
   * @details This is a control error band in which a system's response is
   * insensitive to the magnitude of the error. The deadband is a range of
   * values in which a sensor's output is zero. Use this to eliminate undesired
   * control motion.
   * @param input The input sensor data
   * @param deadband The deadband value
   * @return The sensor data after applying the deadband
   * @see https://en.wikipedia.org/wiki/Deadband
   */
  double applyDeadband(double input, double deadband);

  /**
   * @brief Exponential moving average filter for sensor data
   * @details This is a type of infinite impulse response filter that applies
   * weighting factors which decrease exponentially. The weighting for each
   * older datum decreases exponentially, never reaching zero.
   * @param alpha The smoothing factor, which determines the weight given to the
   * current input.
   * @param input The input sensor data to be filtered
   * @return The filtered sensor data
   * @see
   * https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
   */
  double applyEMAFilter(double alpha, double input);

  /**
   * @brief Butterworth filter for sensor data
   * @details This is a type of signal processing filter designed to have as
   * flat a frequency response as possible in the passband.
   * @param cut_off_freq The cutoff frequency of the filter
   * @param dt The sampling time
   * @param input The input sensor data
   * @return The filtered sensor data
   * @see https://en.wikipedia.org/wiki/Butterworth_filter
   */
  double applyButterworthFilter(double cut_off_freq, double dt, double input);

  /**
   * @brief Kalman filter for sensor data
   * @details This is a recursive algorithm for estimating the
   * internal state of a linear dynamic system from a series of noisy
   * measurements. It is an optimal estimator in the sense that it minimizes the
   * estimated error covariance. The Kalman filter is a two-stage algorithm:
   * 1. Prediction: The Kalman filter produces estimates of the current state
   * variables, along with their uncertainties. These estimates are based on
   * system dynamics and on the previous state estimate.
   * 2. Update: The Kalman filter combines the current estimate with the new
   * measurement information to produce an improved estimate of the current
   * state variables along with their uncertainties.
   * @param process_noise The process noise
   * @param measurement_noise The measurement noise
   * @param input The input sensor data
   * @return The filtered sensor data
   * @see https://en.wikipedia.org/wiki/Kalman_filter
   */
  double applyKalmanFilter(double process_noise, double measurement_noise,
                           double input);

  /**
   * @brief Applies a filter to the input sensor data
   * @note This is a templated function to handle both 1D and 6D wrench data
   * (eg. Fx, Fy, Fz, Tx, Ty, Tz). For the 6D case, the 1D wrench filter is
   * applied to each element of the wrench data.
   * @tparam T The wrench data type
   * @param params The filter parameters struct for the wrench data
   * @post wrench is updated with the filtered wrench data
   */
  template <typename T>
  void filterWrench(T& wrench, const FilterWrenchParams& params);

 private:
  // EMA filter
  double prev_value;

  // Butterworth filter
  double x1, x2; /*Input history*/
  double y1, y2; /*Output history*/

  // Kalman filter
  double error_covariance, estimate /*Initial estimate*/;
};
}  // namespace filter_utils