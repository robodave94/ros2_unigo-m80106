/**
 * @file change_id_node.cpp
 * @brief One-shot ROS 2 CLI tool that discovers all USB-serial adapters
 *        matching a given PID:VID, scans every RS-485 bus for Unitree
 *        GO-M8010-6 actuators, and changes a motor's ID.
 *
 * Node parameters:
 *   pidvid          (string, default "0403:6011") — USB PID:VID to match.
 *   target_id       (int,    required)            — Current motor ID to change (0–14).
 *   new_id          (int,    required)            — Desired new motor ID (0–14).
 *   reboot_wait_ms  (int,    default 1000)         — ms to wait for motor reboot after packet.
 *   debug           (bool,   default false)        — Print raw packet hex and full rescan detail.
 *
 * Procedure:
 *   1. Call serial::findSerialMultipleSerialDevicePathsByPIDVID(pidvid) to
 *      obtain every matching serial port path.
 *   2. For each discovered port, open a MotorDriver and call scanMotors()
 *      (brake-ping to IDs 0–14).
 *   3. Print a summary identical to actuator_ping: port path, hardware_id
 *      and every responding motor ID.
 *   4. After the summary, perform safety checks (in order):
 *        • target_id or new_id > 14  → warn and exit.
 *        • target_id == new_id       → warn and exit.
 *        • target_id not found       → warn and exit.
 *        • target_id on >1 port      → warn about duplicates and exit.
 *        • new_id already occupied   → warn and exit.
 *   5. Attempt the ID change via tryChangeMotorId() on the single port
 *      where target_id was discovered; re-scan to verify.
 *
 * NOTE on ID-change support:
 *   The GO-M8010-6 SDK does not expose a dedicated changeMotorId() API.
 *   This implementation uses the raw serial port to send the vendor-
 *   documented "Broadcast + mode=0x10" packet.  If the firmware does not
 *   respond, the function returns false and logs an error.
 *
 *   Raw protocol reference (vendor RS-485 application note):
 *     TX: [0xFE 0xEE] [id_byte (4-bit target | 0x70)] [new_id_byte] [CRC16_lo] [CRC16_hi]
 *     RX: motor re-boots with new ID and responds to the next brake command.
 *
 * Usage:
 *   ros2 run m80106_execs m80106_change_id \
 *       --ros-args -p target_id:=0 -p new_id:=3
 *   ros2 run m80106_execs m80106_change_id \
 *       --ros-args -p pidvid:=0403:6011 -p target_id:=2 -p new_id:=5
 *
 * Debug / troubleshooting:
 *   ros2 run m80106_execs m80106_change_id \
 *       --ros-args -p target_id:=0 -p new_id:=3 -p debug:=true -p reboot_wait_ms:=3000
 *
 *   debug=true logs:
 *     • Raw 6-byte packet in hex with field annotations.
 *     • Full post-reboot scan results (every motor ID that responded).
 *   reboot_wait_ms — increase if the motor needs longer to reboot before re-scan.
 */

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "serial/serial.h"

#include "m80106_lib/motor_driver.hpp"
#include "m80106_lib/motor_types.hpp"

#include "m80106_execs/multi_serial_go8_scanner.hpp"

#include "crc/crc_ccitt.h"

// ─────────────────────────────────────────────────────────────────────────────

namespace {

/**
 * @brief Send the RS-485 ID-change packet and wait for motor reboot.
 *
 * Returns the full post-reboot scanMotors() result so the caller can
 * print a detailed summary and determine success itself.
 * Returns an empty vector if the packet could not be sent.
 *
 * @param driver          Open MotorDriver instance.
 * @param target_id       Current motor ID (0–14).
 * @param new_id          Desired new motor ID (0–14).
 * @param reboot_wait_ms  Milliseconds to wait before re-scanning.
 * @param debug           When true, log the raw packet bytes in hex.
 * @param logger          ROS 2 logger.
 * @return                IDs found on the bus after the reboot wait.
 *                        Empty vector signals a send failure.
 */
std::vector<uint8_t> tryChangeMotorId(m80106::MotorDriver & driver,
                                       uint8_t target_id,
                                       uint8_t new_id,
                                       int reboot_wait_ms,
                                       bool debug,
                                       const rclcpp::Logger & logger)
{
    // Packet layout (6 bytes):
    //   [0]    0xFE  — header byte 0
    //   [1]    0xEE  — header byte 1
    //   [2]    (target_id & 0x0F) | 0x70  — mode field: id nibble + change-ID command (0x7)
    //   [3]    new_id & 0x0F              — new ID payload
    //   [4..5] CRC16 of bytes [0..3] (little-endian)
    uint8_t pkt[6];
    pkt[0] = 0xFE;
    pkt[1] = 0xEE;
    pkt[2] = static_cast<uint8_t>((target_id & 0x0F) | 0x70);
    pkt[3] = static_cast<uint8_t>(new_id & 0x0F);

    const uint16_t crc = crc_ccitt(0xFFFF, pkt, 4);
    pkt[4] = static_cast<uint8_t>(crc & 0xFF);
    pkt[5] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    if (debug) {
        char hex[32];
        std::snprintf(hex, sizeof(hex),
            "%02X %02X %02X %02X %02X %02X",
            pkt[0], pkt[1], pkt[2], pkt[3], pkt[4], pkt[5]);
        RCLCPP_INFO(logger,
            "[DEBUG] ID-change packet (6 bytes): %s", hex);
        RCLCPP_INFO(logger,
            "[DEBUG]   byte[2] = 0x%02X  (target_id=%d nibble | 0x70 change-ID mode)",
            pkt[2], target_id);
        RCLCPP_INFO(logger,
            "[DEBUG]   byte[3] = 0x%02X  (new_id=%d)",
            pkt[3], new_id);
        RCLCPP_INFO(logger,
            "[DEBUG]   CRC16   = 0x%04X  (lo=0x%02X hi=0x%02X)",
            crc, pkt[4], pkt[5]);
    }

    RCLCPP_INFO(logger, "Sending ID-change packet: motor %d → %d", target_id, new_id);
    const std::size_t sent = driver.serialPort().send(pkt, sizeof(pkt));
    if (sent != sizeof(pkt)) {
        RCLCPP_ERROR(logger, "Failed to send ID-change packet (%zu/%zu bytes).",
                     sent, sizeof(pkt));
        return {};
    }

    RCLCPP_INFO(logger, "Waiting for motor reboot (%d ms)...", reboot_wait_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(reboot_wait_ms));

    RCLCPP_INFO(logger, "Re-scanning bus...");
    const std::vector<uint8_t> found = driver.scanMotors();

    if (debug) {
        if (found.empty()) {
            RCLCPP_INFO(logger, "[DEBUG] Post-reboot scan: no motors responded.");
        } else {
            std::string id_list;
            for (uint8_t id : found) {
                if (!id_list.empty()) id_list += ", ";
                id_list += std::to_string(static_cast<int>(id));
            }
            RCLCPP_INFO(logger,
                "[DEBUG] Post-reboot scan: %zu motor(s) responded: [%s]",
                found.size(), id_list.c_str());
        }
    }

    return found;
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────

namespace m80106_execs {

class ChangeIdNode : public rclcpp::Node
{
public:
    explicit ChangeIdNode(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions())
    : rclcpp::Node("m80106_change_id", opts)
    {
        declare_parameter<std::string>("pidvid",         "0403:6011");
        declare_parameter<int>("target_id",    -1);
        declare_parameter<int>("new_id",       -1);
        declare_parameter<int>("reboot_wait_ms", 1000);
        declare_parameter<bool>("debug",        false);

        const std::string pidvid          = get_parameter("pidvid").as_string();
        const int         target_id_i     = get_parameter("target_id").as_int();
        const int         new_id_i        = get_parameter("new_id").as_int();
        const int         reboot_wait_ms  = get_parameter("reboot_wait_ms").as_int();
        const bool        debug           = get_parameter("debug").as_bool();

        // ── Require both IDs to be explicitly provided ────────────────────
        if (target_id_i < 0) {
            RCLCPP_FATAL(get_logger(),
                "Parameter 'target_id' is required. "
                "Use --ros-args -p target_id:=<0-14>");
            return;
        }
        if (new_id_i < 0) {
            RCLCPP_FATAL(get_logger(),
                "Parameter 'new_id' is required. "
                "Use --ros-args -p new_id:=<0-14>");
            return;
        }

        // ── Discover all matching ports ───────────────────────────────────
        RCLCPP_INFO(get_logger(),
                    "Searching for serial ports matching PID:VID '%s' ...",
                    pidvid.c_str());

        const auto scan = m80106_execs::scanAllPorts(pidvid);

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

        // ── Summary (mirrors actuator_ping output) ────────────────────────
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

        // ── Build lookup maps from scan results ───────────────────────────
        // results: port -> motor_ids  (for compatibility with existing checks)
        std::map<std::string, std::vector<uint8_t>> results;
        std::vector<std::string> ports;
        for (const auto & ps : scan.ports) {
            ports.push_back(ps.port);
            results[ps.port] = ps.motor_ids;
        }

        // ── Post-summary safety checks ────────────────────────────────────

        if (target_id_i > 14) {
            RCLCPP_WARN(get_logger(),
                "This node only supports changing IDs up to 14. "
                "target_id=%d is out of range.",
                target_id_i);
            return;
        }
        if (new_id_i > 14) {
            RCLCPP_WARN(get_logger(),
                "This node only supports changing IDs up to 14. "
                "new_id=%d is out of range.",
                new_id_i);
            return;
        }
        if (target_id_i == new_id_i) {
            RCLCPP_WARN(get_logger(),
                "target_id == new_id (%d). Nothing to do.", target_id_i);
            return;
        }

        const auto target_id = static_cast<uint8_t>(target_id_i);
        const auto new_id    = static_cast<uint8_t>(new_id_i);

        // Collect every port that contains target_id
        std::vector<std::string> ports_with_target;
        for (const std::string & port : ports) {
            for (uint8_t id : results[port]) {
                if (id == target_id) {
                    ports_with_target.push_back(port);
                    break;
                }
            }
        }

        if (ports_with_target.empty()) {
            RCLCPP_WARN(get_logger(),
                "No actuator with target_id=%d found on any scanned port.",
                target_id_i);
            return;
        }

        if (ports_with_target.size() > 1) {
            RCLCPP_WARN(get_logger(),
                "Actuator with target_id=%d was found on %zu different USB paths:",
                target_id_i, ports_with_target.size());
            for (const auto & p : ports_with_target) {
                RCLCPP_WARN(get_logger(), "  %s", p.c_str());
            }
            RCLCPP_WARN(get_logger(),
                "Cannot safely change ID when duplicates exist on different USB paths. "
                "Aborting.");
            return;
        }

        const std::string & target_port = ports_with_target[0];

        // Ensure new_id is free on the target port
        for (uint8_t id : results[target_port]) {
            if (id == new_id) {
                RCLCPP_WARN(get_logger(),
                    "new_id=%d is already occupied by another motor on %s. "
                    "Choose a different ID.",
                    new_id_i, target_port.c_str());
                return;
            }
        }

        // ── Attempt ID change ─────────────────────────────────────────────
        RCLCPP_INFO(get_logger(),
                    "═════════════════════════════════════════════════════");
        RCLCPP_INFO(get_logger(),
                    "Changing motor ID %d → %d on port %s",
                    target_id_i, new_id_i, target_port.c_str());
        RCLCPP_INFO(get_logger(),
                    "═════════════════════════════════════════════════════");

        if (debug) {
            RCLCPP_INFO(get_logger(),
                "[DEBUG] debug=true  reboot_wait_ms=%d", reboot_wait_ms);
        }

        m80106::MotorDriver driver(target_port);
        const std::vector<uint8_t> post_ids =
            tryChangeMotorId(driver, target_id, new_id,
                             reboot_wait_ms, debug, get_logger());

        // ── Post-change summary ───────────────────────────────────────────
        RCLCPP_INFO(get_logger(),
                    "═════════════════════════════════════════════════════");
        RCLCPP_INFO(get_logger(), "POST-CHANGE SCAN  (port: %s)",
                    target_port.c_str());
        RCLCPP_INFO(get_logger(),
                    "═════════════════════════════════════════════════════");

        if (post_ids.empty() && !debug) {
            // empty can mean send failed (debug would have already logged it)
            RCLCPP_WARN(get_logger(), "  No motors responded on the bus.");
        } else {
            for (uint8_t id : post_ids) {
                RCLCPP_INFO(get_logger(),
                    "  [PRESENT] Actuator ID %d on %s", id, target_port.c_str());
            }
            if (post_ids.empty()) {
                RCLCPP_WARN(get_logger(), "  No motors responded on the bus.");
            }
        }

        bool new_id_present    = false;
        bool target_id_present = false;
        for (uint8_t id : post_ids) {
            if (id == new_id)    { new_id_present    = true; }
            if (id == target_id) { target_id_present = true; }
        }

        RCLCPP_INFO(get_logger(),
                    "─────────────────────────────────────────────────────");

        if (new_id_present && !target_id_present) {
            RCLCPP_INFO(get_logger(),
                "SUCCESS: Motor ID changed from %d to %d.",
                target_id_i, new_id_i);
        } else if (new_id_present && target_id_present) {
            RCLCPP_WARN(get_logger(),
                "PARTIAL: new_id=%d is present but old target_id=%d is ALSO still present. "
                "Check for duplicate motors or a partial reboot.",
                new_id_i, target_id_i);
        } else if (!new_id_present && target_id_present) {
            RCLCPP_ERROR(get_logger(),
                "FAILED: Motor still responds at old ID %d; new_id=%d not found.\n"
                "The packet may not be supported by this firmware.\n"
                "Try --ros-args -p debug:=true -p reboot_wait_ms:=3000 for more detail.",
                target_id_i, new_id_i);
        } else {
            RCLCPP_ERROR(get_logger(),
                "FAILED: Motor %d did not respond at new_id=%d after the change attempt.\n"
                "Possible causes:\n"
                "  • Your firmware version does not support runtime ID changes via this packet.\n"
                "  • Use the vendor 'MotorID_Set' utility (Windows) to change the ID offline.\n"
                "  • Verify the motor is operational (power, RS-485 wiring).\n"
                "  • Try --ros-args -p debug:=true -p reboot_wait_ms:=3000 for more detail.",
                target_id_i, new_id_i);
        }
    }

};

}  // namespace m80106_execs

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<m80106_execs::ChangeIdNode>();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}
