#ifndef MY_ROBOT_HARDWARE_HPP
#define MY_ROBOT_HARDWARE_HPP

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

class MyRobotHardware : public hardware_interface::SystemInterface {
public:
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;
    hardware_interface::return_type start() override;
    hardware_interface::return_type stop() override;
    hardware_interface::return_type read() override;
    hardware_interface::return_type write() override;
};

#endif // MY_ROBOT_HARDWARE_HPP
