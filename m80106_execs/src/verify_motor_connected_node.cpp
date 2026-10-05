/**
 * @file verify_motor_connected_node.cpp
 * @brief One-shot gate node for the Pi hardware service: confirms the Unitree
 *        RS-485 adapter and racking-pin motor are present before the caller
 *        proceeds to launch the rest of the stack.
 *
 * Runs three MANDATORY checks on a 1 Hz timer:
 *   1. A serial port with hardware_id matching `pidvid` is enumerated.
 *   2. `serial_port` exists on disk AND is the SAME device that matched
 *      `pidvid` (pairing check — not just "some port matches" and
 *      "the path exists" independently).
 *   3. `motor_id` responds to a brake-ping on that port.
 *
 * While any check fails, an error string is published on /swlq_motor_errors
 * at rate_hz describing the first failing check. Once all three pass, the
 * node logs success and exits 0 (rclcpp::shutdown()) — callers (the hardware
 * service script) treat a non-zero/timeout exit as "not ready" and a clean
 * exit as "ready, proceed to launch".
 *
 * Node parameters:
 *   pidvid       (string, default "1a86:55d3") — USB PID:VID of the RS-485 adapter.
 *   serial_port  (string, default "/dev/ttyACM0") — expected fixed device path.
 *   motor_id     (int,    default 2) — racking-pin motor id to confirm.
 *   rate_hz      (double, default 1.0) — error-publish / recheck rate.
 *   timeout_s    (double, default 0.0) — give up and exit 1 after this many
 *                seconds of continuous failure; 0 = retry forever.
 *
 * Usage:
 *   ros2 run m80106_execs verify_motor_connected
 *   ros2 run m80106_execs verify_motor_connected --ros-args -p motor_id:=2
 */

#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "serial/serial.h"

#include "m80106_lib/motor_driver.hpp"

namespace m80106_execs {

class VerifyMotorConnectedNode : public rclcpp::Node
{
public:
    explicit VerifyMotorConnectedNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
    : rclcpp::Node("verify_motor_connected", opts)
    {
        pidvid_      = declare_parameter<std::string>("pidvid", "1a86:55d3");
        serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        motor_id_    = static_cast<uint8_t>(declare_parameter<int>("motor_id", 2));
        rate_hz_     = declare_parameter<double>("rate_hz", 1.0);
        timeout_s_   = declare_parameter<double>("timeout_s", 0.0);

        rate_hz_ = std::max(0.1, rate_hz_);

        error_pub_ = create_publisher<std_msgs::msg::String>("/swlq_motor_errors", rclcpp::QoS(10));

        start_time_ = now();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz_),
            std::bind(&VerifyMotorConnectedNode::onTimer, this));

        RCLCPP_INFO(get_logger(),
            "verify_motor_connected: checking pidvid=%s serial_port=%s motor_id=%d",
            pidvid_.c_str(), serial_port_.c_str(), static_cast<int>(motor_id_));
    }

private:
    // Checks are evaluated in order; the first failure is reported so the
    // operator always sees the earliest unmet precondition.
    bool checkPidVidPresent(std::string & matched_port) const
    {
        for (const auto & info : serial::list_ports()) {
            if (info.hardware_id.find(pidvid_) != std::string::npos) {
                matched_port = info.port;
                return true;
            }
        }
        return false;
    }

    bool checkPortPaired(const std::string & matched_port) const
    {
        struct stat st{};
        if (stat(serial_port_.c_str(), &st) != 0) return false;
        // Pairing: the port that matched pidvid must BE serial_port, not just
        // some other device that happens to also match the PID:VID.
        return matched_port == serial_port_;
    }

    bool checkMotorResponds() const
    {
        try {
            m80106::MotorDriver driver(serial_port_);
            MotorData data;
            return driver.brake(motor_id_, data) && data.correct;
        } catch (const std::exception &) {
            return false;
        }
    }

    void onTimer()
    {
        std::string matched_port;
        const bool pidvid_ok = checkPidVidPresent(matched_port);
        const bool paired_ok = pidvid_ok && checkPortPaired(matched_port);
        const bool motor_ok  = paired_ok && checkMotorResponds();

        if (pidvid_ok && paired_ok && motor_ok) {
            RCLCPP_INFO(get_logger(),
                "verify_motor_connected: all checks passed (port=%s, motor %d responded). Exiting 0.",
                serial_port_.c_str(), static_cast<int>(motor_id_));
            rclcpp::shutdown();
            return;
        }

        std::string reason;
        if (!pidvid_ok) {
            reason = "required usb device not connected on pi (pidvid " + pidvid_ + " not found)";
        } else if (!paired_ok) {
            reason = "required usb device not connected on pi (" + serial_port_ +
                      " missing or not paired with pidvid " + pidvid_ + ")";
        } else {
            reason = "required usb device not connected on pi (motor id " +
                      std::to_string(static_cast<int>(motor_id_)) + " not responding on " +
                      serial_port_ + ")";
        }

        std_msgs::msg::String msg;
        msg.data = reason;
        error_pub_->publish(msg);
        RCLCPP_WARN(get_logger(), "%s", reason.c_str());

        if (timeout_s_ > 0.0 && (now() - start_time_).seconds() > timeout_s_) {
            RCLCPP_ERROR(get_logger(), "verify_motor_connected: timed out after %.1fs. Exiting 1.", timeout_s_);
            exit_code_ = 1;
            rclcpp::shutdown();
        }
    }

public:
    int exitCode() const noexcept { return exit_code_; }

private:
    std::string pidvid_;
    std::string serial_port_;
    uint8_t motor_id_ = 2;
    double rate_hz_ = 1.0;
    double timeout_s_ = 0.0;
    rclcpp::Time start_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
    int exit_code_ = 0;
};

}  // namespace m80106_execs

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<m80106_execs::VerifyMotorConnectedNode>();
    rclcpp::spin(node);
    const int rc = node->exitCode();
    rclcpp::shutdown();
    return rc;
}
