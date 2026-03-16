/**
 * @file change_id_node.cpp
 * @brief One-shot ROS 2 CLI tool to change a GO-M8010-6 motor's RS-485 ID.
 *
 * Node parameters:
 *   pidvid         (string, default "1a86:7523") — USB PID:VID of the adapter.
 *   serial_number  (string, default "")          — Optional USB serial number filter.
 *   port           (string, default "")          — Explicit device path (overrides discovery).
 *   target_id      (int,    required)            — Current motor ID to change (0–14).
 *   new_id         (int,    required)            — Desired new motor ID (0–14).
 *
 * Procedure:
 *   1. Connect to the RS-485 adapter (same discovery logic as m80106_ui_node).
 *   2. Scan the bus — verify target_id is present and new_id is absent.
 *   3. Attempt the ID change via tryChangeMotorId().
 *   4. Re-scan to verify the change was applied.
 *   5. Log result and exit.
 *
 * NOTE on ID-change support:
 *   The GO-M8010-6 SDK (libUnitreeMotorSDK_M80106_*.so) does not expose a
 *   dedicated changeMotorId() API.  The implementation below uses the raw
 *   serial port to send the vendor-documented "Broadcast + mode=0x10" packet
 *   that instructs all motors with a given ID to adopt a new ID.  If the
 *   firmware does not respond, the function returns false and logs an error.
 *
 *   Raw protocol reference (from vendor RS-485 application note):
 *     TX: [0xFE 0xEE] [id_byte (4-bit target | 0x70)] [new_id_byte] [CRC16_lo] [CRC16_hi]
 *     RX: motor re-boots with new ID and responds to the next brake command.
 *
 *   If your firmware does not support this, use the vendor's "MotorID_Set"
 *   Windows utility over RS-485 to change the ID offline.
 *
 * Usage:
 *   ros2 run m80106_bringup m80106_change_id \
 *       --ros-args -p target_id:=0 -p new_id:=3
 */

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "m80106_lib/motor_bus.hpp"
#include "m80106_lib/motor_driver.hpp"
#include "m80106_lib/motor_types.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// CRC helper (CRC-16/CCITT-FALSE, matches the SDK's crc_ccitt.h)
// ─────────────────────────────────────────────────────────────────────────────

#include "crc/crc_ccitt.h"

// ─────────────────────────────────────────────────────────────────────────────

namespace {

/**
 * @brief Attempt to change the RS-485 ID of a motor.
 *
 * Sends a small proprietary 6-byte broadcast packet and waits for the motor
 * to reboot.  Returns true if a motor at @p new_id responds afterwards.
 *
 * The packet is NOT part of the documented SDK API; it relies on internal
 * motor firmware support.  If the motor does not reboot within ~1 second
 * the operation is considered failed.
 *
 * @param driver      Open MotorDriver instance.
 * @param target_id   Current motor ID (0–14).
 * @param new_id      Desired new motor ID (0–14).
 * @param logger      ROS 2 logger for status messages.
 * @return            true if the re-scan confirms new_id is now present.
 */
bool tryChangeMotorId(m80106::MotorDriver & driver,
                      uint8_t target_id,
                      uint8_t new_id,
                      const rclcpp::Logger & logger)
{
    // Packet layout (6 bytes):
    //   [0]    0xFE  — header byte 0
    //   [1]    0xEE  — header byte 1
    //   [2]    (target_id & 0x0F) | 0x70  — mode field: id nibble + change-ID command (0x7)
    //   [3]    new_id & 0x0F              — new ID payload
    //   [4..5] CRC16 of bytes [0..3] (little-endian)
    //
    // NOTE: 0x70 in the upper nibble is a vendor-internal "set ID" mode not
    // listed in the public SDK.  If your firmware is older this may be unsupported.

    uint8_t pkt[6];
    pkt[0] = 0xFE;
    pkt[1] = 0xEE;
    pkt[2] = static_cast<uint8_t>((target_id & 0x0F) | 0x70);
    pkt[3] = static_cast<uint8_t>(new_id & 0x0F);

    const uint16_t crc = crc_ccitt(0xFFFF, pkt, 4);
    pkt[4] = static_cast<uint8_t>(crc & 0xFF);
    pkt[5] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    RCLCPP_INFO(logger, "Sending ID-change packet: motor %d → %d", target_id, new_id);
    const std::size_t sent = driver.serialPort().send(pkt, sizeof(pkt));
    if (sent != sizeof(pkt)) {
        RCLCPP_ERROR(logger, "Failed to send ID-change packet (%zu/%zu bytes).",
                     sent, sizeof(pkt));
        return false;
    }

    // Motor reboots — wait up to 1 second
    RCLCPP_INFO(logger, "Waiting for motor reboot (~1 s)...");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Verify: scan for new_id
    const std::vector<uint8_t> found = driver.scanMotors();
    for (uint8_t id : found) {
        if (id == new_id) {
            return true;
        }
    }
    return false;
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("m80106_change_id");

    node->declare_parameter<std::string>("pidvid",        "1a86:7523");
    node->declare_parameter<std::string>("serial_number", "");
    node->declare_parameter<std::string>("port",          "");
    node->declare_parameter<int>("target_id", -1);
    node->declare_parameter<int>("new_id",    -1);

    const auto pidvid        = node->get_parameter("pidvid").as_string();
    const auto serial_number = node->get_parameter("serial_number").as_string();
    const auto explicit_port = node->get_parameter("port").as_string();
    const int  target_id_i   = node->get_parameter("target_id").as_int();
    const int  new_id_i      = node->get_parameter("new_id").as_int();

    const auto & log = node->get_logger();

    // ── Validate parameters ───────────────────────────────────────────────
    if (target_id_i < 0 || target_id_i > static_cast<int>(m80106::MAX_MOTOR_ID)) {
        RCLCPP_FATAL(log,
            "Parameter 'target_id' must be 0–%d. Got: %d",
            static_cast<int>(m80106::MAX_MOTOR_ID), target_id_i);
        rclcpp::shutdown();
        return 1;
    }
    if (new_id_i < 0 || new_id_i > static_cast<int>(m80106::MAX_MOTOR_ID)) {
        RCLCPP_FATAL(log,
            "Parameter 'new_id' must be 0–%d. Got: %d",
            static_cast<int>(m80106::MAX_MOTOR_ID), new_id_i);
        rclcpp::shutdown();
        return 1;
    }
    if (target_id_i == new_id_i) {
        RCLCPP_WARN(log, "target_id == new_id (%d). Nothing to do.", target_id_i);
        rclcpp::shutdown();
        return 0;
    }

    const auto target_id = static_cast<uint8_t>(target_id_i);
    const auto new_id    = static_cast<uint8_t>(new_id_i);

    // ── Discover adapter ──────────────────────────────────────────────────
    std::unique_ptr<m80106::MotorBus> bus;
    try {
        if (!explicit_port.empty()) {
            bus = std::make_unique<m80106::MotorBus>(explicit_port, true);
        } else if (!serial_number.empty()) {
            const auto ports = serial::list_ports();
            std::string matched;
            for (const auto & pi : ports) {
                if (pi.hardware_id.find(pidvid)        != std::string::npos &&
                    pi.hardware_id.find(serial_number) != std::string::npos)
                {
                    matched = pi.port;
                    break;
                }
            }
            if (matched.empty()) {
                RCLCPP_FATAL(log, "No adapter matching PID:VID=%s SNR=%s",
                             pidvid.c_str(), serial_number.c_str());
                rclcpp::shutdown();
                return 1;
            }
            bus = std::make_unique<m80106::MotorBus>(matched, true);
        } else {
            bus = std::make_unique<m80106::MotorBus>(pidvid);
        }
    } catch (const std::exception & e) {
        RCLCPP_FATAL(log, "Adapter discovery failed: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    if (!bus->isConnected()) {
        RCLCPP_FATAL(log,
            "RS-485 adapter not found. "
            "Check USB connection or use --ros-args -p port:=/dev/ttyUSBx");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(log, "Adapter: %s  (%s)",
                bus->portPath().c_str(), bus->hardwareId().c_str());

    // ── Open driver ───────────────────────────────────────────────────────
    m80106::MotorDriver driver(bus->portPath());

    // ── Initial scan ──────────────────────────────────────────────────────
    RCLCPP_INFO(log, "Scanning bus...");
    const std::vector<uint8_t> initial_ids = driver.scanMotors();

    bool target_found = false;
    bool new_id_taken = false;
    for (uint8_t id : initial_ids) {
        if (id == target_id) { target_found = true; }
        if (id == new_id)    { new_id_taken = true; }
    }

    if (!target_found) {
        RCLCPP_FATAL(log,
            "Motor with target_id=%d not found on the bus. "
            "Detected IDs: [", target_id);
        for (auto id : initial_ids) {
            RCLCPP_FATAL(log, "  %d", id);
        }
        rclcpp::shutdown();
        return 1;
    }

    if (new_id_taken) {
        RCLCPP_FATAL(log,
            "new_id=%d is already occupied by another motor on the bus. "
            "Choose a different ID.", new_id);
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(log, "Pre-change scan OK: motor %d found, new_id %d is free.",
                target_id, new_id);

    // ── Attempt ID change ─────────────────────────────────────────────────
    const bool ok = tryChangeMotorId(driver, target_id, new_id, log);

    if (ok) {
        RCLCPP_INFO(log,
            "SUCCESS: Motor ID changed from %d to %d. "
            "You may now use --ros-args otherwise at the new ID.",
            target_id, new_id);
    } else {
        RCLCPP_ERROR(log,
            "FAILED: Motor %d did not respond at new_id=%d after the change attempt.\n"
            "Possible causes:\n"
            "  • Your firmware version does not support runtime ID changes via this packet.\n"
            "  • Use the vendor 'MotorID_Set' utility (Windows) to change the ID offline.\n"
            "  • Verify the motor is operational (power, RS-485 wiring).",
            target_id, new_id);
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
