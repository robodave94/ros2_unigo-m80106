#pragma once
/**
 * @file multi_serial_go8_scanner.hpp
 * @brief Shared multi-port GO-M8010-6 motor scanner.
 *
 * Iterates every USB-serial adapter matching a given PID:VID, opens the
 * RS-485 bus, and pings motor IDs 0–14 via brake commands.  Results are
 * returned in an abstractable struct for further processing by any node.
 *
 * Usage:
 * @code
 *   auto result = m80106_execs::scanAllPorts("0403:6011");
 *   for (const auto & port : result.ports) {
 *       for (uint8_t id : port.motor_ids) { ... }
 *   }
 * @endcode
 */

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "serial/serial.h"

#include "m80106_lib/motor_driver.hpp"
#include "m80106_lib/motor_types.hpp"

namespace m80106_execs {

// ─────────────────────────────────────────────────────────────────────────────
// Data types
// ─────────────────────────────────────────────────────────────────────────────

/// A single motor discovered on a bus.
struct DiscoveredMotor {
    uint8_t     id;
    std::string port;
    std::string hardware_id;
};

/// Scan result for one serial port.
struct PortScanResult {
    std::string          port;
    std::string          hardware_id;
    std::vector<uint8_t> motor_ids;
};

/// Aggregate scan result across all matching serial ports.
struct MultiScanResult {
    std::vector<PortScanResult> ports;

    /// Total number of motors found across all ports.
    size_t totalMotors() const noexcept {
        size_t n = 0;
        for (const auto & p : ports) n += p.motor_ids.size();
        return n;
    }

    /// Flatten all discovered motors into a single list.
    std::vector<DiscoveredMotor> allMotors() const {
        std::vector<DiscoveredMotor> out;
        for (const auto & p : ports) {
            for (uint8_t id : p.motor_ids) {
                out.push_back({id, p.port, p.hardware_id});
            }
        }
        return out;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Scanner functions
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Discover all serial ports matching @p pidvid.
 *
 * Returns port paths and their hardware_id strings.  No motor communication
 * happens here — use scanPort() or scanAllPorts() for that.
 *
 * @param pidvid  PID:VID string, e.g. "0403:6011".
 * @return        Vector of (port_path, hardware_id) pairs.  Empty on error.
 */
inline std::vector<std::pair<std::string, std::string>>
discoverPorts(const std::string & pidvid)
{
    std::vector<std::pair<std::string, std::string>> result;

    std::vector<std::string> paths;
    try {
        paths = serial::findSerialMultipleSerialDevicePathsByPIDVID(pidvid);
    } catch (const std::runtime_error &) {
        return result;
    }

    // Build a quick lookup of hardware_id by port path
    const auto all_ports = serial::list_ports();
    std::map<std::string, std::string> hw_map;
    for (const auto & info : all_ports) {
        hw_map[info.port] = info.hardware_id;
    }

    for (const auto & path : paths) {
        auto it = hw_map.find(path);
        std::string hw = (it != hw_map.end()) ? it->second : "n/a";
        result.emplace_back(path, hw);
    }
    return result;
}

/**
 * @brief Scan a single serial port for responding motors.
 *
 * Opens the port, sends brake-ping to IDs 0–MAX_MOTOR_ID, returns those
 * that responded.
 *
 * @param port         Device path, e.g. "/dev/ttyUSB0".
 * @param hardware_id  Hardware ID string (for populating the result).
 * @return             PortScanResult with discovered motor IDs.
 */
inline PortScanResult
scanPort(const std::string & port, const std::string & hardware_id = "n/a")
{
    PortScanResult res;
    res.port        = port;
    res.hardware_id = hardware_id;

    try {
        m80106::MotorDriver driver(port);
        res.motor_ids = driver.scanMotors();
    } catch (const std::exception &) {
        // Port could not be opened — return empty motor list
    }
    return res;
}

/**
 * @brief Discover all matching serial ports and scan each for motors.
 *
 * This is the primary entry point.  Combines discoverPorts() + scanPort()
 * into a single call.
 *
 * @param pidvid  PID:VID string, e.g. "0403:6011".
 * @return        MultiScanResult with per-port results.
 */
inline MultiScanResult
scanAllPorts(const std::string & pidvid)
{
    MultiScanResult result;
    const auto discovered = discoverPorts(pidvid);

    for (const auto & [port, hw_id] : discovered) {
        result.ports.push_back(scanPort(port, hw_id));
    }
    return result;
}

}  // namespace m80106_execs
