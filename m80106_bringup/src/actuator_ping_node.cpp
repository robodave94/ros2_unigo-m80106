/**
 * @file actuator_ping_node.cpp
 * @brief One-shot ROS 2 CLI tool that discovers all USB-serial adapters
 *        matching a given PID:VID and pings every Unitree GO-M8010-6
 *        actuator found on each RS-485 bus.
 *
 * Node parameters:
 *   pidvid  (string, default "0403:6011") — USB PID:VID to match.
 *                                           All ports whose hardware_id
 *                                           contains this string are scanned.
 *
 * Procedure:
 *   1. Call serial::findSerialMultipleSerialDevicePathsByPIDVID(pidvid) to
 *      obtain every matching serial port path.
 *   2. For each discovered port, open a MotorDriver and call scanMotors()
 *      (brake-ping to IDs 0–14).
 *   3. Print a summary: port path, hardware_id and every responding motor ID.
 *
 * Usage:
 *   ros2 run m80106_bringup actuator_ping
 *   ros2 run m80106_bringup actuator_ping --ros-args -p pidvid:=0403:6011
 */

#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "serial/serial.h"

#include "m80106_lib/motor_driver.hpp"
#include "m80106_lib/motor_types.hpp"

namespace m80106_bringup {

class ActuatorPingNode : public rclcpp::Node
{
public:
    explicit ActuatorPingNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
    : rclcpp::Node("actuator_ping", opts)
    {
        declare_parameter<std::string>("pidvid", "0403:6011");
        const std::string pidvid = get_parameter("pidvid").as_string();

        RCLCPP_INFO(get_logger(), "Searching for serial ports matching PID:VID '%s' ...",
                    pidvid.c_str());

        // ── Discover all matching ports ───────────────────────────────────
        std::vector<std::string> ports;
        try {
            ports = serial::findSerialMultipleSerialDevicePathsByPIDVID(pidvid);
        } catch (const std::runtime_error & e) {
            RCLCPP_ERROR(get_logger(), "No ports found for PID:VID '%s': %s",
                         pidvid.c_str(), e.what());
            return;
        }

        RCLCPP_INFO(get_logger(), "Found %zu port(s) matching '%s'.",
                    ports.size(), pidvid.c_str());

        // ── Ping each port ────────────────────────────────────────────────
        std::map<std::string, std::vector<uint8_t>> results;
        for (const std::string & port : ports) {
            results[port] = pingPort(port);
        }

        // ── Summary ───────────────────────────────────────────────────────
        RCLCPP_INFO(get_logger(),
                    "═════════════════════════════════════════════════════");
        RCLCPP_INFO(get_logger(), "SUMMARY  (%zu port(s) scanned)", ports.size());
        RCLCPP_INFO(get_logger(),
                    "═════════════════════════════════════════════════════");

        size_t total_motors = 0;
        for (const std::string & port : ports) {
            const auto & ids = results[port];
            total_motors += ids.size();
            if (ids.empty()) {
                RCLCPP_INFO(get_logger(), "  %-20s  → no actuators", port.c_str());
            } else {
                std::string id_list;
                for (uint8_t id : ids) {
                    if (!id_list.empty()) id_list += ", ";
                    id_list += std::to_string(static_cast<int>(id));
                }
                RCLCPP_INFO(get_logger(), "  %-20s  → %zu actuator(s): [%s]",
                            port.c_str(), ids.size(), id_list.c_str());
            }
        }
        RCLCPP_INFO(get_logger(),
                    "─────────────────────────────────────────────────────");
        RCLCPP_INFO(get_logger(),
                    "Total: %zu actuator(s) across %zu port(s).",
                    total_motors, ports.size());
    }

private:
    std::vector<uint8_t> pingPort(const std::string & port)
    {
        // Enrich log output with hardware_id from sysfs if available
        std::string hw_id = "n/a";
        for (const serial::PortInfo & info : serial::list_ports()) {
            if (info.port == port) {
                hw_id = info.hardware_id;
                break;
            }
        }

        RCLCPP_INFO(get_logger(),
                    "─────────────────────────────────────────────────────");
        RCLCPP_INFO(get_logger(), "Port      : %s", port.c_str());
        RCLCPP_INFO(get_logger(), "Hardware  : %s", hw_id.c_str());
        RCLCPP_INFO(get_logger(), "Scanning actuators (IDs 0–%d) ...",
                    static_cast<int>(m80106::MAX_MOTOR_ID));

        // Try to open the port and scan
        std::vector<uint8_t> found;
        try {
            m80106::MotorDriver driver(port);
            found = driver.scanMotors();
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to open %s: %s  (check dialout group membership)",
                         port.c_str(), e.what());
            return found;
        }

        if (found.empty()) {
            RCLCPP_WARN(get_logger(),
                        "No Unitree actuators responded on %s.", port.c_str());
        } else {
            RCLCPP_INFO(get_logger(),
                        "%zu actuator(s) confirmed on %s:",
                        found.size(), port.c_str());
            for (uint8_t id : found) {
                RCLCPP_INFO(get_logger(),
                            "  [PRESENT] Actuator ID %d on %s", id, port.c_str());
            }
        }
        return found;
    }
};

}  // namespace m80106_bringup

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<m80106_bringup::ActuatorPingNode>();
    // Single-shot: spin briefly to process one round of callbacks then exit.
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
