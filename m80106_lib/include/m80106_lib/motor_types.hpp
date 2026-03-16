#pragma once

#include <cstdint>
#include <cmath>

/**
 * @file motor_types.hpp
 * @brief Constants, enumerations, and unit-conversion helpers for the
 *        Unitree GO-M8010-6 motor.
 *
 * Include this header in every node or control algorithm that needs to
 * reason about motor quantities.  All values are rotor-side unless the
 * function name contains "Output" (which refers to the output shaft of
 * the gearbox).
 *
 * Gear-ratio note (from the manufacturer README):
 *   kp_rotor = kp_output / r²     where r = GEAR_RATIO = 6.33
 *   kd_rotor = kd_output / r²
 */

namespace m80106 {

// ─────────────────────────────────────────────────────────────────────────────
// Hardware constants — GO-M8010-6
// ─────────────────────────────────────────────────────────────────────────────

/// Output-to-rotor gear ratio.
constexpr float GEAR_RATIO = 6.33f;

/// Highest individually addressable motor ID on the RS-485 bus.
/// ID 15 (BROADCAST_ID) sends to all motors simultaneously; no response is returned.
constexpr uint8_t MAX_MOTOR_ID = 14;
constexpr uint8_t BROADCAST_ID = 15;

/// Temperature thresholds (°C).  Hardware protection activates at 90 °C.
constexpr int8_t TEMP_WARNING_CELSIUS    = 75;
constexpr int8_t TEMP_PROTECTION_CELSIUS = 90;

/// Physical rotor-side limits as defined by the SDK.
constexpr float MAX_TORQUE_NM  = 127.99f;   ///< ±N·m
constexpr float MAX_SPEED_RADS = 804.00f;   ///< ±rad/s
constexpr float MAX_POS_RAD    = 411774.f;  ///< ±rad

/// PD gain coefficient limits (K_P and K_W in the SDK).
constexpr float MAX_KP = 25.599f;
constexpr float MAX_KD = 25.599f;

// ─────────────────────────────────────────────────────────────────────────────
// Enumerations
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Motor operating mode (maps directly to MotorCmd::mode / MotorData::mode).
 *
 * SDK mode values: 0=BRAKE, 1=FOC, 2=CALIBRATE
 */
enum class MotorMode : uint8_t {
    BRAKE     = 0,  ///< Locked / passive brake.
    FOC       = 1,  ///< FOC closed-loop (torque / velocity / position).
    CALIBRATE = 2,  ///< Encoder calibration.  Do not send other commands for ≥5 s.
};

/**
 * @brief Decoded error flags from MotorData::MError.
 */
enum class MotorError : uint8_t {
    NONE          = 0,
    OVERHEAT      = 1,
    OVERCURRENT   = 2,
    OVERVOLTAGE   = 3,
    ENCODER_FAULT = 4,
};

// ─────────────────────────────────────────────────────────────────────────────
// Conversion helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Cast MotorMode to the uint16_t field expected by MotorCmd::mode.
inline uint16_t toSDKMode(MotorMode m) noexcept
{
    return static_cast<uint16_t>(m);
}

/// Decode a raw SDK MError integer into the MotorError enum.
inline MotorError toMotorError(int e) noexcept
{
    if (e >= 0 && e <= 4) {
        return static_cast<MotorError>(e);
    }
    return MotorError::NONE;
}

/// Human-readable error string.
inline const char * errorString(MotorError e) noexcept
{
    switch (e) {
        case MotorError::NONE:          return "OK";
        case MotorError::OVERHEAT:      return "Overheat";
        case MotorError::OVERCURRENT:   return "Overcurrent";
        case MotorError::OVERVOLTAGE:   return "Overvoltage";
        case MotorError::ENCODER_FAULT: return "Encoder Fault";
    }
    return "Unknown";
}

/// Human-readable mode string.
inline const char * modeString(MotorMode m) noexcept
{
    switch (m) {
        case MotorMode::BRAKE:     return "BRAKE";
        case MotorMode::FOC:       return "FOC";
        case MotorMode::CALIBRATE: return "CALIBRATE";
    }
    return "Unknown";
}

/// Convert output-side Kp to rotor-side Kp  (kp_rotor = kp_output / r²).
inline float toRotorKp(float kp_output) noexcept
{
    return kp_output / (GEAR_RATIO * GEAR_RATIO);
}

/// Convert output-side Kd to rotor-side Kd  (kd_rotor = kd_output / r²).
inline float toRotorKd(float kd_output) noexcept
{
    return kd_output / (GEAR_RATIO * GEAR_RATIO);
}

/// Convert output-shaft position [rad] to rotor position [rad].
inline float toRotorPos(float output_pos_rad) noexcept
{
    return output_pos_rad * GEAR_RATIO;
}

/// Convert rotor position [rad] to output-shaft position [rad].
inline float toOutputPos(float rotor_pos_rad) noexcept
{
    return rotor_pos_rad / GEAR_RATIO;
}

/// Convert output-shaft speed [rad/s] to rotor speed [rad/s].
inline float toRotorSpeed(float output_speed) noexcept
{
    return output_speed * GEAR_RATIO;
}

/// Convert rotor speed [rad/s] to output-shaft speed [rad/s].
inline float toOutputSpeed(float rotor_speed) noexcept
{
    return rotor_speed / GEAR_RATIO;
}

/// Saturate @p v between @p lo and @p hi.
template <typename T>
inline T clamp(T v, T lo, T hi) noexcept
{
    return v < lo ? lo : (v > hi ? hi : v);
}

} // namespace m80106
