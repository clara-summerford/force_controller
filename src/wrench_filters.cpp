#include "force_controller/wrench_filters.hpp"
#include "force_controller/controller_utils.hpp"

namespace filter_utils {

FilterType stringToFilterType(const std::string& str) {
  static const std::unordered_map<std::string, FilterType> filterTypeMap = {
      {"EMA", FilterType::EMA},
      {"Butterworth", FilterType::Butterworth},
      {"Kalman", FilterType::Kalman}};

  auto it = filterTypeMap.find(str);
  return (it != filterTypeMap.end())
             ? it->second
             : FilterType::EMA;  // Default to EMA if not found
}

// Constructor
FilterUtils::FilterUtils()
    : prev_value(0.0),
      x1(0.0),
      x2(0.0),
      y1(0.0),
      y2(0.0),
      error_covariance(1.0),
      estimate(1.0) {}

// Destructor
FilterUtils::~FilterUtils() {}

double FilterUtils::applyDeadband(double input, double deadband) {
  // Ensure deadband is non-negative
  deadband = std::max(deadband, 0.0);

  // Apply deadband
  return (fabs(input) > deadband) ? input : 0.0;
}

double FilterUtils::applyEMAFilter(double alpha, double input) {
  // Exponential moving average filter
  double output = alpha * input + (1.0 - alpha) * prev_value;
  // Update previous value for the next iteration
  prev_value = output;
  return output;
}

double FilterUtils::applyButterworthFilter(double cut_off_freq, double dt,
                                           double input) {
  // Book: "Digital Signal Processing: Principles, Algorithms, and Applications"
  // by John G. Proakis and Dimitris G. Manolakis.

  // Calculate filter coefficients
  double omegaC = (2.0 / dt) * tan(cut_off_freq * dt / 2.0);
  double a0 = 1.0 + 2.0 * cos(omegaC * dt) + pow(cos(omegaC * dt), 2.0);
  double b0 = pow(cos(omegaC * dt), 2.0) / a0;
  double b1 = 2.0 * b0;
  double b2 = b0;  // b2 = b0 due to filter symmetry
  double a1 = -2.0 * cos(omegaC * dt) / a0;
  double a2 = (1.0 - 2.0 * cos(omegaC * dt) + pow(cos(omegaC * dt), 2.0)) / a0;

  // Apply filter
  double output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;

  // Update state
  x2 = x1;
  x1 = input;
  y2 = y1;
  y1 = output;

  return output;
}

double FilterUtils::applyKalmanFilter(double process_noise,
                                      double measurement_noise, double input) {
  // Prediction
  double predicted_estimate = estimate;
  double predicted_error_covariance = error_covariance + process_noise;

  // Update
  double kalman_gain = predicted_error_covariance /
                       (predicted_error_covariance + measurement_noise);
  estimate = predicted_estimate + kalman_gain * (input - predicted_estimate);
  error_covariance = (1.0 - kalman_gain) * predicted_error_covariance;

  return estimate;
}

template <typename T>
void FilterUtils::filterWrench(T& wrench, const FilterWrenchParams& params) {
  // Apply deadband
  wrench = applyDeadband(wrench, params.deadband_threshold);

  // Apply filters in the specified order
  for (const auto& filter_type : params.filter_order) {
    switch (filter_type) {
      case FilterType::EMA:
        if (params.apply_ema_filter) {
          wrench = applyEMAFilter(params.alpha, wrench);
        }
        break;
      case FilterType::Butterworth:
        if (params.apply_bw_filter) {
          wrench = applyButterworthFilter(params.cut_off_frequency, params.dt,
                                          wrench);
        }
        break;
      case FilterType::Kalman:
        if (params.apply_kf_filter) {
          wrench = applyKalmanFilter(params.process_noise,
                                     params.measurement_noise, wrench);
        }
        break;
      default:
        RCLCPP_WARN(rclcpp::get_logger("controller_utils"),
                    "Unknown filter type");
        break;
    }
  }
}

template <>
void FilterUtils::filterWrench(geometry_msgs::msg::WrenchStamped& wrench,
                               const FilterWrenchParams& params) {
  for (auto& wrenchElement : controller_utils::toWrenchVector(wrench)) {
    filterWrench(wrenchElement, params);
  }
}

}  // namespace filter_utils
