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
 *   1. Use multi_serial_go8_scanner to discover all matching serial ports
 *      and scan for responding motors (brake-ping to IDs 0–14).
 *   2. Print a summary: port path, hardware_id and every responding motor ID.
 *
 * Usage:
 *   ros2 run m80106_execs actuator_ping
 *   ros2 run m80106_execs actuator_ping --ros-args -p pidvid:=0403:6011
 */

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "m80106_lib/multi_serial_go8_scanner.hpp"

namespace m80106_execs {

using m80106::scanAllPorts;

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

        // ── Scan all matching ports ───────────────────────────────────────
        const auto scan = scanAllPorts(pidvid);

        if (scan.ports.empty()) {
            RCLCPP_ERROR(get_logger(), "No ports found for PID:VID '%s'.",
                         pidvid.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Found %zu port(s) matching '%s'.",
                    scan.ports.size(), pidvid.c_str());

        // ── Per-port detail ───────────────────────────────────────────────
        for (const auto & ps : scan.ports) {
            RCLCPP_INFO(get_logger(),
                        "─────────────────────────────────────────────────────");
            RCLCPP_INFO(get_logger(), "Port      : %s", ps.port.c_str());
            RCLCPP_INFO(get_logger(), "Hardware  : %s", ps.hardware_id.c_str());
            RCLCPP_INFO(get_logger(), "Scanning actuators (IDs 0–%d) ...",
                        static_cast<int>(m80106::MAX_MOTOR_ID));

            if (ps.motor_ids.empty()) {
                RCLCPP_WARN(get_logger(),
                            "No Unitree actuators responded on %s.", ps.port.c_str());
            } else {
                RCLCPP_INFO(get_logger(),
                            "%zu actuator(s) confirmed on %s:",
                            ps.motor_ids.size(), ps.port.c_str());
                for (uint8_t id : ps.motor_ids) {
                    RCLCPP_INFO(get_logger(),
                                "  [PRESENT] Actuator ID %d on %s", id, ps.port.c_str());
                }
            }
        }

        // ── Summary ───────────────────────────────────────────────────────
        RCLCPP_INFO(get_logger(),
                    "═════════════════════════════════════════════════════");
        RCLCPP_INFO(get_logger(), "SUMMARY  (%zu port(s) scanned)", scan.ports.size());
        RCLCPP_INFO(get_logger(),
                    "═════════════════════════════════════════════════════");

        for (const auto & ps : scan.ports) {
            if (ps.motor_ids.empty()) {
                RCLCPP_INFO(get_logger(), "  %-20s  → no actuators", ps.port.c_str());
            } else {
                std::string id_list;
                for (uint8_t id : ps.motor_ids) {
                    if (!id_list.empty()) id_list += ", ";
                    id_list += std::to_string(static_cast<int>(id));
                }
                RCLCPP_INFO(get_logger(), "  %-20s  → %zu actuator(s): [%s]",
                            ps.port.c_str(), ps.motor_ids.size(), id_list.c_str());
            }
        }
        RCLCPP_INFO(get_logger(),
                    "─────────────────────────────────────────────────────");
        RCLCPP_INFO(get_logger(),
                    "Total: %zu actuator(s) across %zu port(s).",
                    scan.totalMotors(), scan.ports.size());
    }
};

}  // namespace m80106_execs

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<m80106_execs::ActuatorPingNode>();
    // Single-shot: spin briefly to process one round of callbacks then exit.
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
