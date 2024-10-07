
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "socket_can.hpp"

namespace odrive_ros2_control {

class Axis;

class ODriveHardwareInterface final : public hardware_interface::SystemInterface {
public:
    using return_type = hardware_interface::return_type;
    using State = rclcpp_lifecycle::State;

    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    CallbackReturn on_configure(const State& previous_state) override;
    CallbackReturn on_cleanup(const State& previous_state) override;
    CallbackReturn on_activate(const State& previous_state) override;
    CallbackReturn on_deactivate(const State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces
    ) override;

    return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    void on_can_msg(const can_frame& frame);

    EpollEventLoop event_loop_;
    std::vector<Axis> axes_;
    std::string can_intf_name_;
    SocketCanIntf can_intf_;
    rclcpp::Time timestamp_;
};

struct Axis {
    Axis(SocketCanIntf* can_intf, uint32_t node_id) : can_intf_(can_intf), node_id_(node_id) {}

    void on_can_msg(const rclcpp::Time& timestamp, const can_frame& frame);

    void on_can_msg();

    SocketCanIntf* can_intf_;
    uint32_t node_id_;

    rclcpp::Clock clock_ = {};
    rclcpp::Time timeout_ = {};

    void queue_clear_errors(rclcpp::Duration timeout = rclcpp::Duration::from_seconds(0))
    {
        if (queue_clear_errors_) return;
        timeout_ = clock_.now() + timeout;
        queue_clear_errors_ = true;
    }

    bool queue_clear_errors_ = false;

    // Commands (ros2_control => ODrives)
    double pos_setpoint_ = 0.0f; // [rad]
    double vel_setpoint_ = 0.0f; // [rad/s]
    double torque_setpoint_ = 0.0f; // [Nm]
    double clear_errors_ = 0.0f;

    // State (ODrives => ros2_control)
    // rclcpp::Time encoder_estimates_timestamp_;
    uint32_t axis_error_ = 0;
    uint8_t axis_state_ = 0;
    // uint8_t procedure_result_ = 0;
    // uint8_t trajectory_done_flag_ = 0;
    double pos_estimate_ = NAN; // [rad]
    double vel_estimate_ = NAN; // [rad/s]
    // double iq_setpoint_ = NAN;
    // double iq_measured_ = NAN;
    double torque_target_ = NAN; // [Nm]
    double torque_estimate_ = NAN; // [Nm]
    uint32_t active_errors_ = 0;
    uint32_t disarm_reason_ = 0;
    double fet_temperature_ = NAN;
    double motor_temperature_ = NAN;
    double bus_voltage_ = NAN;
    double bus_current_ = NAN;
    double error_ = 0.0f;
    double state_ = 0.0f;

    // Indicates which controller inputs are enabled. This is configured by the
    // controller that sits on top of this hardware interface. Multiple inputs
    // can be enabled at the same time, in this case the non-primary inputs are
    // used as feedforward terms.
    // This implicitly defines the ODrive's control mode.
    bool pos_input_enabled_ = false;
    bool vel_input_enabled_ = false;
    bool torque_input_enabled_ = false;

    void handle_queue()
    {
        if (queue_clear_errors_ && clock_.now() >= timeout_)
        {
            Set_Controller_Mode_msg_t msg;
            if (pos_input_enabled_) {
                RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %d to position control", node_id_);
                msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
                msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
            } else if (vel_input_enabled_) {
                RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %d to velocity control", node_id_);
                msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
                msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
            } else {
                RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %d to torque control", node_id_);
                msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
                msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
            }

            bool any_enabled = pos_input_enabled_ || vel_input_enabled_ || torque_input_enabled_;

            if (any_enabled) {
                send(msg); // Set control mode
            }
            // Set axis state
            Clear_Errors_msg_t msg1;
            msg1.Identify = 0;
            send(msg1);

            // Set axis state
            Set_Axis_State_msg_t msg2;
            msg2.Axis_Requested_State = any_enabled ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE;
            send(msg2);
        }
    }

    template <typename T>
    void send(const T& msg) {
        struct can_frame frame;
        frame.can_id = node_id_ << 5 | msg.cmd_id;
        frame.can_dlc = msg.msg_length;
        msg.encode_buf(frame.data);

        can_intf_->send_can_frame(frame);
    }
};

} // namespace odrive_ros2_control

using namespace odrive_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    can_intf_name_ = info_.hardware_parameters["can"];

    for (auto& joint : info_.joints) {
        axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")));
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_configure(const State&) {
    if (!can_intf_.init(can_intf_name_, &event_loop_, std::bind(&ODriveHardwareInterface::on_can_msg, this, _1))) {
        RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"), "Failed to initialize SocketCAN on %s", can_intf_name_.c_str());
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Initialized SocketCAN on %s", can_intf_name_.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_cleanup(const State&) {
    can_intf_.deinit();
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_activate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "activating ODrives...");

    // This can be called several seconds before the controller finishes starting.
    // Therefore we enable the ODrives only in perform_command_mode_switch().
    return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const State&) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "deactivating ODrives...");

    for (auto& axis : axes_) {
        Set_Axis_State_msg_t msg;
        msg.Axis_Requested_State = AXIS_STATE_IDLE;
        axis.send(msg);
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_target_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_estimate_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            "voltage",
            &axes_[i].bus_voltage_
        ));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name,
            "current",
            &axes_[i].bus_current_
        ));
        // state_interfaces.emplace_back(hardware_interface::StateInterface(
        //     info_.joints[i].name,
        //     "error",
        //     &axes_[i].axis_error_
        // ));
        // state_interfaces.emplace_back(hardware_interface::StateInterface(
        //     info_.joints[i].name,
        //     "state",
        //     &axes_[i].axis_state_
        // ));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ODriveHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,
            &axes_[i].torque_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY,
            &axes_[i].vel_setpoint_
        ));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &axes_[i].pos_setpoint_
        ));
    }

    return command_interfaces;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
) {
    for (size_t i = 0; i < axes_.size(); ++i) {
        Axis& axis = axes_[i];
        std::array<std::pair<std::string, bool*>, 3> interfaces = {
            {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION, &axis.pos_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, &axis.vel_input_enabled_},
             {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT, &axis.torque_input_enabled_}}
        };

        bool mode_switch = false;

        for (const std::string& key : stop_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = false;
                    mode_switch = true;
                }
            }
        }

        for (const std::string& key : start_interfaces) {
            for (auto& kv : interfaces) {
                if (kv.first == key) {
                    *kv.second = true;
                    mode_switch = true;
                }
            }
        }

        if (mode_switch) {
            Set_Controller_Mode_msg_t msg;
            if (axis.pos_input_enabled_) {
                RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %s to position control", info_.joints[i].name.c_str());
                msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
                msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
            } else if (axis.vel_input_enabled_) {
                RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %s to velocity control", info_.joints[i].name.c_str());
                msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
                msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
            } else {
                RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"), "Setting %s to torque control", info_.joints[i].name.c_str());
                msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
                msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
            }

            bool any_enabled = axis.pos_input_enabled_ || axis.vel_input_enabled_ || axis.torque_input_enabled_;

            if (any_enabled) {
                axis.send(msg); // Set control mode
            }

            // Set axis state
            Clear_Errors_msg_t msg1;
            msg1.Identify = 0;
            axis.send(msg1);

            // Set axis state
            Set_Axis_State_msg_t msg2;
            msg2.Axis_Requested_State = any_enabled ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE;
            axis.send(msg2);
        }
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time& timestamp, const rclcpp::Duration&) {
    static rclcpp::Clock clk = rclcpp::Clock();
    timestamp_ = timestamp;

    while (can_intf_.read_nonblocking()) {
        // repeat until CAN interface has no more messages
    }

    for (auto& axis : axes_)
    {
        axis.error_ = (double)axis.axis_error_;
        axis.state_ = (double)axis.axis_state_;
        switch (axis.axis_error_)
        {
            case ODRIVE_ERROR_SYSTEM_LEVEL:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_SYSTEM_LEVEL for node_id: ", axis.node_id_);
                axis.queue_clear_errors();
                break;
            case ODRIVE_ERROR_TIMING_ERROR:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_TIMING_ERROR for node_id: ", axis.node_id_);
                axis.queue_clear_errors();
                break;
            case ODRIVE_ERROR_MISSING_ESTIMATE:
                RCLCPP_FATAL_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_MISSING_ESTIMATE for node_id: ", axis.node_id_);
                return return_type::ERROR;
            case ODRIVE_ERROR_BAD_CONFIG:
                RCLCPP_FATAL_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_BAD_CONFIG for node_id: ", axis.node_id_);
                return return_type::ERROR;
            case ODRIVE_ERROR_DRV_FAULT:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_DRV_FAULT for node_id: ", axis.node_id_);
                axis.queue_clear_errors();
                break;
            case ODRIVE_ERROR_MISSING_INPUT:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_MISSING_INPUT for node_id: ", axis.node_id_);
                axis.queue_clear_errors();
                break;
            case ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE for node_id: ", axis.node_id_);
                axis.queue_clear_errors(rclcpp::Duration(std::chrono::seconds(5)));
                break;
            case ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE:
                RCLCPP_FATAL_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE for node_id: ", axis.node_id_);
                return return_type::ERROR;
            case ODRIVE_ERROR_DC_BUS_OVER_CURRENT:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_DC_BUS_OVER_CURRENT for node_id: ", axis.node_id_);
                axis.queue_clear_errors(rclcpp::Duration(std::chrono::seconds(5)));
                break;
            case ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT for node_id: ", axis.node_id_);
                axis.queue_clear_errors(rclcpp::Duration(std::chrono::seconds(5)));
                break;
            case ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION for node_id: ", axis.node_id_);
                axis.queue_clear_errors(rclcpp::Duration(std::chrono::seconds(5)));
                break;
            case ODRIVE_ERROR_MOTOR_OVER_TEMP:
                RCLCPP_FATAL_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_MOTOR_OVER_TEMP for node_id: ", axis.node_id_);
                return return_type::ERROR;
            case ODRIVE_ERROR_INVERTER_OVER_TEMP:
                RCLCPP_FATAL_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_INVERTER_OVER_TEMP for node_id: ", axis.node_id_);
                return return_type::ERROR;
            case ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION for node_id: ", axis.node_id_);
                axis.queue_clear_errors(rclcpp::Duration(std::chrono::seconds(1)));
                break;
            case ODRIVE_ERROR_POSITION_LIMIT_VIOLATION:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_POSITION_LIMIT_VIOLATION for node_id: ", axis.node_id_);
                axis.queue_clear_errors(rclcpp::Duration(std::chrono::seconds(1)));
                break;
            case ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED for node_id: ", axis.node_id_);
                axis.queue_clear_errors();
                break;
            case ODRIVE_ERROR_ESTOP_REQUESTED:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_ESTOP_REQUESTED for node_id: ", axis.node_id_);
                // axis.queue_clear_errors();
                break;
            case ODRIVE_ERROR_SPINOUT_DETECTED:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_SPINOUT_DETECTED for node_id: ", axis.node_id_);
                axis.queue_clear_errors(rclcpp::Duration(std::chrono::seconds(5)));
                break;
            case ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED for node_id: ", axis.node_id_);
                break;
            case ODRIVE_ERROR_THERMISTOR_DISCONNECTED:
                RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_THERMISTOR_DISCONNECTED for node_id: ", axis.node_id_);
                break;
            case ODRIVE_ERROR_CALIBRATION_ERROR:
                RCLCPP_FATAL_THROTTLE(rclcpp::get_logger("ODriveHardwareInterface"), clk, 2000, "ODRIVE_ERROR_CALIBRATION_ERROR for node_id: ", axis.node_id_);
                return return_type::ERROR;
            case ODRIVE_ERROR_NONE:
            case ODRIVE_ERROR_INITIALIZING:
                break;
            default:
                break;
        }
    }

    return return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    for (auto& axis : axes_) {
        
        if (axis.clear_errors_ > 0.5f)
        {
            axis.clear_errors_ = 0.0f;
            axis.queue_clear_errors();
        }

        axis.handle_queue();

        // Send the CAN message that fits the set of enabled setpoints
        if (axis.pos_input_enabled_) {
            Set_Input_Pos_msg_t msg;
            msg.Input_Pos = axis.pos_setpoint_ / (2 * M_PI);
            msg.Vel_FF = axis.vel_input_enabled_ ? (axis.vel_setpoint_  / (2 * M_PI)) : 0.0f;
            msg.Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            axis.send(msg);
        } else if (axis.vel_input_enabled_) {
            Set_Input_Vel_msg_t msg;
            msg.Input_Vel = axis.vel_setpoint_ / (2 * M_PI);
            msg.Input_Torque_FF = axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
            axis.send(msg);
        } else if (axis.torque_input_enabled_) {
            Set_Input_Torque_msg_t msg;
            msg.Input_Torque = axis.torque_setpoint_;
            axis.send(msg);
        } else {
            // no control enabled - don't send any setpoint
        }
    }

    return return_type::OK;
}

void ODriveHardwareInterface::on_can_msg(const can_frame& frame) {
    for (auto& axis : axes_) {
        if ((frame.can_id >> 5) == axis.node_id_) {
            axis.on_can_msg(timestamp_, frame);
        }
    }
}

void Axis::on_can_msg(const rclcpp::Time&, const can_frame& frame) {
    uint8_t cmd = frame.can_id & 0x1f;

    auto try_decode = [&]<typename TMsg>(TMsg& msg) {
        if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
            RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"), "message %d too short", cmd);
            return false;
        }
        msg.decode_buf(frame.data);
        return true;
    };

    switch (cmd) {
        case Get_Encoder_Estimates_msg_t::cmd_id: {
            if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
                pos_estimate_ = msg.Pos_Estimate * (2 * M_PI);
                vel_estimate_ = msg.Vel_Estimate * (2 * M_PI);
            }
        } break;
        case Get_Torques_msg_t::cmd_id: {
            if (Get_Torques_msg_t msg; try_decode(msg)) {
                torque_target_ = msg.Torque_Target;
                torque_estimate_ = msg.Torque_Estimate;
            }
        } break;
        case Heartbeat_msg_t::cmd_id: {
            if (Heartbeat_msg_t msg; try_decode(msg)) {
                axis_error_ = msg.Axis_Error;
                axis_state_ = msg.Axis_State;
            }
        } break;
        case Get_Bus_Voltage_Current_msg_t::cmd_id: {
            if (Get_Bus_Voltage_Current_msg_t msg; try_decode(msg)){
                bus_voltage_ = msg.Bus_Voltage;
                bus_current_ = msg.Bus_Current;
            }
        }
    }
}

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface, hardware_interface::SystemInterface)
