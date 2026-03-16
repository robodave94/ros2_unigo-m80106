#pragma once

#include <string>
#include <vector>
#include <stdexcept>

// serial/serial.h is provided by the serial-ros2 package (build dep: serial).
// Downstream executables must also link against libserial.
#include "serial/serial.h"
#include "m80106_lib/motor_types.hpp"

/**
 * @file motor_bus.hpp
 * @brief RS-485 bus adapter discovery using the serial-ros2 PID:VID API.
 *
 * MotorBus is a lightweight discovery object.  It locates the USB-to-RS485
 * adapter by PID:VID (using serial::findSerialDevicePathByPIDVID) or accepts
 * an explicit port path and does NOT own the serial port file descriptor —
 * that is the responsibility of MotorDriver.
 *
 * Default PID:VID: CH340/CH341 USB-serial adapter  →  "1a86:7523"
 *
 * Usage — auto-discover:
 * @code
 *   m80106::MotorBus bus("1a86:7523");
 *   if (!bus.isConnected()) {
 *     RCLCPP_ERROR(logger, "RS-485 adapter not found");
 *   } else {
 *     MotorDriver driver(bus.portPath());
 *   }
 * @endcode
 *
 * Usage — explicit port:
 * @code
 *   m80106::MotorBus bus("/dev/ttyUSB0", true);
 * @endcode
 *
 * @note Requires the serial package to be present in ROS 2 workspace.
 *       Add `<depend>serial</depend>` to your package.xml and
 *       `find_package(serial REQUIRED)` + link against serial in CMakeLists.
 */

namespace m80106 {

/// Describes the physical RS-485 bus adapter.
struct BusInfo {
    std::string port_path;    ///< Device path, e.g. "/dev/ttyUSB0"
    std::string hardware_id;  ///< From sysfs, e.g. "USB VID:PID=1a86:7523 SNR=..."
    bool        connected = false;
};

/**
 * @brief Discovers and reports the RS-485 adapter port.
 *
 * Does NOT open the port.  Pass MotorBus::portPath() to MotorDriver.
 */
class MotorBus
{
public:
    /**
     * @brief Auto-discover the adapter by PID:VID.
     *
     * Uses serial::findSerialDevicePathByPIDVID() from serial-ros2.
     * On failure (adapter not plugged in), connected() returns false; no
     * exception is thrown.
     *
     * @param pidvid  PID:VID string, e.g. "1a86:7523" for CH340/CH341.
     */
    explicit MotorBus(const std::string & pidvid = "1a86:7523")
    {
        try {
            info_.port_path = serial::findSerialDevicePathByPIDVID(pidvid);
            // Enrich with the full hardware_id string (VID:PID + serial number)
            for (const auto & port : serial::list_ports()) {
                if (port.port == info_.port_path) {
                    info_.hardware_id = port.hardware_id;
                    break;
                }
            }
            info_.connected = true;
        } catch (const std::exception &) {
            info_.connected = false;
        }
    }

    /**
     * @brief Accept an explicit port path (skips PID:VID discovery).
     *
     * The second bool parameter avoids ambiguity with the pidvid overload.
     * Assumes the port exists; MotorDriver will fail if it does not.
     *
     * @param port_path     Device path, e.g. "/dev/ttyUSB0".
     * @param explicit_path Disambiguation tag — pass `true`.
     */
    MotorBus(const std::string & port_path, bool /*explicit_path*/)
    {
        info_.port_path = port_path;
        for (const auto & port : serial::list_ports()) {
            if (port.port == port_path) {
                info_.hardware_id = port.hardware_id;
                break;
            }
        }
        info_.connected = true;  // let MotorDriver detect errors on open
    }

    /// Full bus information struct.
    const BusInfo &   info()       const noexcept { return info_; }

    /// Whether a matching adapter was found.
    bool              isConnected() const noexcept { return info_.connected; }

    /// Device file path (empty string if not connected).
    const std::string & portPath()   const noexcept { return info_.port_path; }

    /// Sysfs hardware_id string, e.g. "USB VID:PID=1a86:7523 SNR=...".
    const std::string & hardwareId() const noexcept { return info_.hardware_id; }

private:
    BusInfo info_;
};

} // namespace m80106
