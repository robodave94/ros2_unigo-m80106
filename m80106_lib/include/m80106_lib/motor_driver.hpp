#pragma once

#include <cstdint>
#include <vector>
#include <string>

// SDK headers — provided via sdk_include/ bundled in m80106_lib.
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

#include "m80106_lib/motor_types.hpp"

/**
 * @file motor_driver.hpp
 * @brief Typed C++ wrapper for the Unitree GO-M8010-6 RS-485 communication.
 *
 * MotorDriver owns the SerialPort (from the bundled Unitree SDK) and exposes
 * a typed API for sending commands and receiving motor feedback.  One
 * MotorDriver instance per RS-485 bus (port); it can address every motor
 * on that bus (IDs 0–14) using the `id` field of MotorCmd.
 *
 * ─── SDK field quick-reference (all quantities are rotor-side) ───────────
 *
 *   MotorCmd field  │ Unit    │ Range           │ Description
 *   ────────────────┼─────────┼─────────────────┼─────────────────────────
 *   id              │ int     │ 0–14  (15=bcast) │ Motor address
 *   mode            │ uint16  │ 0/1/2           │ BRAKE / FOC / CALIBRATE
 *   T               │ N·m     │ ±127.99         │ Desired torque
 *   W               │ rad/s   │ ±804.00         │ Desired speed
 *   Pos             │ rad     │ ±411774         │ Desired position
 *   K_P             │ —       │ 0–25.599        │ Position stiffness
 *   K_W             │ —       │ 0–25.599        │ Velocity damping
 *
 *   MotorData field │ Unit    │ Description
 *   ────────────────┼─────────┼─────────────────────────────────────────────
 *   correct         │ bool    │ Data integrity (CRC + ID valid)
 *   motor_id        │ uint8   │ Responding motor ID
 *   mode            │ uint8   │ BRAKE=0 / FOC=1 / CALIBRATE=2
 *   T               │ N·m     │ Measured torque
 *   W               │ rad/s   │ Measured speed
 *   Pos             │ rad     │ Measured position
 *   Temp            │ °C      │ Motor temperature  (protection at 90 °C)
 *   MError          │ int     │ Error flags (see MotorError enum)
 *   footForce       │ ADC     │ Foot pressure sensor (0–4095)
 *
 * ─── Usage example ──────────────────────────────────────────────────────
 * @code
 *   m80106::MotorDriver driver("/dev/ttyUSB0");
 *
 *   MotorCmd  cmd;
 *   MotorData fb;
 *   cmd.motorType = MotorType::GO_M8010_6;
 *   cmd.id        = 0;
 *   cmd.mode      = m80106::toSDKMode(m80106::MotorMode::FOC);
 *   cmd.K_P       = m80106::toRotorKp(10.0f);
 *   cmd.K_W       = m80106::toRotorKd(1.0f);
 *   cmd.Pos       = m80106::toRotorPos(1.5f);  // 1.5 rad output-side
 *   cmd.W         = 0.0f;
 *   cmd.T         = 0.0f;
 *
 *   if (driver.sendRecv(cmd, fb) && fb.correct) {
 *     float output_pos = m80106::toOutputPos(fb.Pos);
 *   }
 * @endcode
 */

namespace m80106 {

class MotorDriver
{
public:
    /**
     * @brief Open the RS-485 serial port.
     *
     * @param port_path   Device path, e.g. "/dev/ttyUSB0".
     * @param baudrate    Baud rate.  GO-M8010-6 requires 4 000 000 bps.
     * @param timeout_us  Per-message receive timeout in microseconds.
     */
    explicit MotorDriver(
        const std::string & port_path,
        uint32_t            baudrate   = 4000000,
        size_t              timeout_us = 20000)
    : serial_(port_path, /*recvLength=*/16, baudrate, timeout_us)
    {}

    // ─────────────────────────────────────────────────────────────────────
    // Core communicate
    // ─────────────────────────────────────────────────────────────────────

    /**
     * @brief Send a MotorCmd and receive the corresponding MotorData.
     *
     * This is a single-bus-transaction (send 17 bytes, receive 16 bytes).
     * The call blocks for at most `timeout_us` microseconds.
     *
     * @param[in]  cmd   Command to send (caller must set motorType, id,
     *                   mode and all setpoint fields).
     * @param[out] data  Received feedback.  Check data.correct before use.
     * @return           true if data.correct is true.
     */
    bool sendRecv(MotorCmd & cmd, MotorData & data)
    {
        return serial_.sendRecv(&cmd, &data);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Convenience helpers
    // ─────────────────────────────────────────────────────────────────────

    /**
     * @brief Send a brake (zero-setpoint) command to motor @p id.
     *
     * @param[in]  id    Motor ID (0–MAX_MOTOR_ID).
     * @param[out] data  Feedback (check data.correct).
     * @return           true if motor responded with valid feedback.
     */
    bool brake(uint8_t id, MotorData & data)
    {
        MotorCmd cmd;
        cmd.motorType = MotorType::GO_M8010_6;
        cmd.id        = id;
        cmd.mode      = toSDKMode(MotorMode::BRAKE);
        cmd.T         = 0.0f;
        cmd.W         = 0.0f;
        cmd.Pos       = 0.0f;
        cmd.K_P       = 0.0f;
        cmd.K_W       = 0.0f;
        return sendRecv(cmd, data);
    }

    /**
     * @brief Scan the RS-485 bus for all responding motor IDs.
     *
     * Sends a brake command to each ID in [0, MAX_MOTOR_ID] and records
     * those that respond with valid (correct=true) feedback.
     *
     * Worst-case scan time ≈ 15 × timeout_us (default ≈ 300 ms).
     *
     * @return Vector of motor IDs (values 0–14) that responded.
     */
    std::vector<uint8_t> scanMotors()
    {
        std::vector<uint8_t> found;
        MotorData data;
        for (uint8_t id = 0; id <= MAX_MOTOR_ID; ++id) {
            if (brake(id, data) && data.correct) {
                found.push_back(id);
            }
        }
        return found;
    }

    /**
     * @brief Direct access to the underlying SerialPort for advanced use.
     *
     * Use this if you need the raw send/recv byte API, or to reconfigure
     * baud rate at runtime via serialPort().resetSerial(...).
     */
    SerialPort & serialPort() noexcept { return serial_; }

private:
    SerialPort serial_;
};

} // namespace m80106
