#include "sbus_bridge/thrust_mapping.h"

#include <quadrotor_common/parameter_helper.h>
#include <ros/ros.h>

namespace thrust_mapping {

CollectiveThrustMapping::CollectiveThrustMapping()
    : thrust_map_a_p_(0.0),
      thrust_map_b_p_(0.0),
      thrust_map_c_p_(0.0),
      thrust_map_a_n_(0.0),
      thrust_map_b_n_(0.0),
      thrust_map_c_n_(0.0),
      perform_thrust_voltage_compensation_(false),
      thrust_ratio_voltage_map_a_(0.0),
      thrust_ratio_voltage_map_b_(0.0),
      n_lipo_cells_(0) {}

CollectiveThrustMapping::CollectiveThrustMapping(
    const double thrust_map_a_p, const double thrust_map_b_p,
    const double thrust_map_c_p,
    const double thrust_map_a_n, const double thrust_map_b_n,
    const double thrust_map_c_n, const bool perform_thrust_voltage_compensation,
    const double thrust_ratio_voltage_map_a,
    const double thrust_ratio_voltage_map_b, const int n_lipo_cells)
    : thrust_map_a_p_(thrust_map_a_p),
      thrust_map_b_p_(thrust_map_b_p),
      thrust_map_c_p_(thrust_map_c_p),
      thrust_map_a_n_(thrust_map_a_n),
      thrust_map_b_n_(thrust_map_b_n),
      thrust_map_c_n_(thrust_map_c_n),
      perform_thrust_voltage_compensation_(perform_thrust_voltage_compensation),
      thrust_ratio_voltage_map_a_(thrust_ratio_voltage_map_a),
      thrust_ratio_voltage_map_b_(thrust_ratio_voltage_map_b),
      n_lipo_cells_(n_lipo_cells) {}

CollectiveThrustMapping::~CollectiveThrustMapping() {}

uint16_t CollectiveThrustMapping::inverseThrustMapping(
    const double thrust, const double battery_voltage) const {
  double thrust_applied = thrust;
  if (perform_thrust_voltage_compensation_) {
    if (battery_voltage >=
            n_lipo_cells_ * kMinBatteryCompensationVoltagePerCell_ &&
        battery_voltage <=
            n_lipo_cells_ * kMaxBatteryCompensationVoltagePerCell_) {
      const double thrust_cmd_voltage_ratio =
          thrust_ratio_voltage_map_a_ * battery_voltage +
          thrust_ratio_voltage_map_b_;
      thrust_applied *= thrust_cmd_voltage_ratio;
    } else {
      ROS_WARN_THROTTLE(1.0,
                        "[%s] Battery voltage out of range for compensation",
                        ros::this_node::getName().c_str());
    }
  }

  //Citardauq Formula: Gives a numerically stable solution of the quadratic equation for thrust_map_a ~ 0, which is not the case for the standard formula.
  // const uint16_t cmd = 2.0 * (thrust_map_c_ - thrust_applied) / (-thrust_map_b_ - sqrt(thrust_map_b_ * thrust_map_b_ - 4.0 * thrust_map_a_ * (thrust_map_c_ - thrust_applied)));
  uint16_t cmd;
  if (thrust_applied > 0.0){
    cmd = 2.0 * (thrust_map_c_p_ - thrust_applied) / (-thrust_map_b_p_ - sqrt(thrust_map_b_p_ * thrust_map_b_p_ - 4.0 * thrust_map_a_p_ * (thrust_map_c_p_ - thrust_applied)));
  } else if (thrust_applied < 0.0){
    cmd = 2.0 * (thrust_map_c_n_ - thrust_applied) / (-thrust_map_b_n_ + sqrt(thrust_map_b_n_ * thrust_map_b_n_ - 4.0 * thrust_map_a_n_ * (thrust_map_c_n_ - thrust_applied)));
  } else {
    cmd = 992;
  }

  return cmd;
}

bool CollectiveThrustMapping::loadParameters() {
  ros::NodeHandle pnh("~");

#define GET_PARAM(name) \
  if (!quadrotor_common::getParam(#name, name##_, pnh)) return false

  GET_PARAM(thrust_map_a_p);
  GET_PARAM(thrust_map_b_p);
  GET_PARAM(thrust_map_c_p);
  GET_PARAM(thrust_map_a_n);
  GET_PARAM(thrust_map_b_n);
  GET_PARAM(thrust_map_c_n);

  GET_PARAM(perform_thrust_voltage_compensation);
  GET_PARAM(thrust_ratio_voltage_map_a);
  GET_PARAM(thrust_ratio_voltage_map_b);
  GET_PARAM(n_lipo_cells);

  return true;

#undef GET_PARAM
}

}  // namespace thrust_mapping
