#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "odrive_ros2_control/msg/o_drive_joint_state.hpp"
#include "odrive_ros2_control/srv/request_clear_errors.hpp"
#include "odrive_ros2_control/srv/request_reboot.hpp"
#include "odrive_can/srv/axis_state.hpp"

namespace odrive_ros2_control {

class ODriveControllerInterface : public controller_interface::ControllerInterface
{
public:
    controller_interface::CallbackReturn on_init() override
    {
        // Initialize the publisher
        voltage_pub_ = get_node()->create_publisher<std_msgs::msg::Float64>("flipper_voltage", 1);
        current_pub_ = get_node()->create_publisher<std_msgs::msg::Float64>("flipper_current", 1);
        avg_voltage_pub_ = get_node()->create_publisher<std_msgs::msg::Float64>("average_voltage", 1);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration command_interface_configuration() const override
    {
        
    }

    controller_interface::return_type update_and_write_commands(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // Access the state interfaces (e.g., voltage and current) from the hardware interface
        for (auto& v : state_interfaces_)
        {
        }
        double voltage = state_interfaces_[0].get_value();  // Assuming state_interfaces_[0] is for voltage
        double current = state_interfaces_[1].get_value();  // Assuming state_interfaces_[1] is for current

        // Publish voltage
        std_msgs::msg::Float64 voltage_msg;
        voltage_msg.data = voltage;
        voltage_pub_->publish(voltage_msg);

        // Publish current
        std_msgs::msg::Float64 current_msg;
        current_msg.data = current;
        current_pub_->publish(current_msg);

        return controller_interface::return_type::OK;
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr avg_voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_pub_;
};
}