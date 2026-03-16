/**
 * @file m80106_ui_node.cpp
 * @brief ROS 2 node that launches the htop-style ncurses TUI for
 *        interactive GO-M8010-6 motor management over RS-485.
 *
 * Node parameters (all optional):
 *   pidvid         (string, default "1a86:7523")  — USB PID:VID of the
 *                  RS-485 adapter (CH340/CH341).
 *   serial_number  (string, default "")           — Optional USB serial
 *                  number to disambiguate when multiple adapters are
 *                  plugged in.
 *   port           (string, default "")           — Explicit device path
 *                  (e.g. "/dev/ttyUSB1"). When non-empty, overrides
 *                  PID:VID discovery.
 *
 * The ROS 2 executor runs in a background thread so that parameter
 * introspection and node lifecycle are functional while FTXUI's
 * ScreenInteractive owns the main thread.  No topics or services are
 * published by this node; all interaction is through the TUI.
 *
 * Usage:
 *   ros2 run m80106_bringup m80106_ui_node
 *   ros2 run m80106_bringup m80106_ui_node --ros-args -p pidvid:=1a86:7523
 *   ros2 run m80106_bringup m80106_ui_node --ros-args -p port:=/dev/ttyUSB1
 */

#include <atomic>
#include <csignal>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"

#include "m80106_lib/motor_bus.hpp"
#include "m80106_lib/motor_driver.hpp"
#include "m80106_lib/motor_types.hpp"

#include "m80106_bringup/app_state.hpp"
#include "m80106_bringup/tui.hpp"

// ─────────────────────────────────────────────────────────────────────────────

static std::atomic<bool> g_shutdown{false};

static void sigintHandler(int /*sig*/)
{
    g_shutdown.store(true);
    rclcpp::shutdown();
}

// ─────────────────────────────────────────────────────────────────────────────

namespace m80106_bringup {

class M80106UINode : public rclcpp::Node
{
public:
    explicit M80106UINode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
    : rclcpp::Node("m80106_ui", opts)
    {
        declare_parameter<std::string>("pidvid",        "1a86:7523");
        declare_parameter<std::string>("serial_number", "");
        declare_parameter<std::string>("port",          "");

        const auto pidvid         = get_parameter("pidvid").as_string();
        const auto serial_number  = get_parameter("serial_number").as_string();
        const auto explicit_port  = get_parameter("port").as_string();

        // ── Discover or accept explicit port ─────────────────────────────
        if (!explicit_port.empty()) {
            RCLCPP_INFO(get_logger(), "Using explicit port: %s", explicit_port.c_str());
            bus_ = std::make_unique<m80106::MotorBus>(explicit_port, /*explicit_path=*/true);
        } else if (!serial_number.empty()) {
            // Attempt to find adapter matching both PID:VID and serial number.
            // serial::findSerialMultipleSerialDevicePathsByPIDVID returns a vector;
            // we iterate and match the iSerial substring.
            const auto ports = serial::list_ports();
            std::string matched_port;
            for (const auto & pi : ports) {
                if (pi.hardware_id.find(pidvid) != std::string::npos &&
                    pi.hardware_id.find(serial_number) != std::string::npos)
                {
                    matched_port = pi.port;
                    break;
                }
            }
            if (matched_port.empty()) {
                RCLCPP_FATAL(get_logger(),
                    "No RS-485 adapter found matching PID:VID=%s and serial=%s",
                    pidvid.c_str(), serial_number.c_str());
                throw std::runtime_error("RS-485 adapter not found");
            }
            RCLCPP_INFO(get_logger(), "Found adapter: %s", matched_port.c_str());
            bus_ = std::make_unique<m80106::MotorBus>(matched_port, /*explicit_path=*/true);
        } else {
            RCLCPP_INFO(get_logger(), "Auto-discovering RS-485 adapter (PID:VID %s)...",
                        pidvid.c_str());
            bus_ = std::make_unique<m80106::MotorBus>(pidvid);
        }

        if (!bus_->isConnected()) {
            RCLCPP_FATAL(get_logger(),
                "RS-485 adapter not found. "
                "Check USB connection, PID:VID parameter, or use --ros-args -p port:=/dev/ttyUSBx");
            throw std::runtime_error("RS-485 adapter not connected");
        }

        RCLCPP_INFO(get_logger(), "RS-485 adapter: %s  (%s)",
                    bus_->portPath().c_str(),
                    bus_->hardwareId().c_str());

        driver_ = std::make_unique<m80106::MotorDriver>(bus_->portPath());
    }

    m80106::MotorDriver & driver() { return *driver_; }
    const m80106::BusInfo & busInfo() const { return bus_->info(); }

private:
    std::unique_ptr<m80106::MotorBus>    bus_;
    std::unique_ptr<m80106::MotorDriver> driver_;
};

} // namespace m80106_bringup

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, sigintHandler);

    // ── Construct node (discovers RS-485 adapter) ────────────────────────
    std::shared_ptr<m80106_bringup::M80106UINode> node;
    try {
        node = std::make_shared<m80106_bringup::M80106UINode>();
    } catch (const std::exception & e) {
        // Error already logged inside the constructor; clean up and exit.
        rclcpp::shutdown();
        return 1;
    }

    // ── Spin in a background thread ──────────────────────────────────────
    // The ncurses TUI must own the main thread (POSIX signal constraints).
    std::thread spin_thread([&node]() {
        rclcpp::spin(node);
    });

    // Suppress ROS 2 log output while the FTXUI TUI is active so that
    // console prints don't corrupt the full-screen terminal UI.
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);

    // ── Launch TUI ───────────────────────────────────────────────────────
    // FTXUI's ScreenInteractive manages terminal raw mode internally;
    // no explicit init/shutdown pair is required.
    m80106_bringup::AppState   state;
    m80106_bringup::TuiApp     tui;
    int ret = 0;

    try {
        tui.run(node->driver(), node->busInfo(), state);
    } catch (const std::exception & e) {
        rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_INFO);
        fprintf(stderr, "[m80106_ui] TUI error: %s\n", e.what());
        ret = 1;
    }

    // ── Teardown ─────────────────────────────────────────────────────────
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_INFO);
    rclcpp::shutdown();
    spin_thread.join();
    return ret;
}
