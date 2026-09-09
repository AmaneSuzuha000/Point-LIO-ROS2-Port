// Reproduces the two -Wdeprecated diagnostics that the rule-order bug produced:
//   (a) Livox rename ran AFTER the ConstPtr rule  -> CustomMsg::ConstPtr survived
//   (b) no rule for non-const ::Ptr               -> Imu::Ptr survived
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
void a(const livox_ros_driver2::msg::CustomMsg::ConstPtr &m) { (void)m; }
void b() { sensor_msgs::msg::Imu::Ptr p(new sensor_msgs::msg::Imu()); (void)p; }
