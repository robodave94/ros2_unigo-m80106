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
 *
 * Important — protocol limits vs physical limits:
 *   The SDK encodes setpoints in Q8/Q15 fixed-point integers whose
 *   representable ranges (e.g. ±127.99 N·m for torque) are far larger
 *   than the motor's actual physical capabilities (≈23.7 N·m peak at
 *   the output shaft).  Both sets of constants are provided below:
 *     - PROTOCOL_MAX_*  — encoding limits for SDK serialisation clamps.
 *     - OUTPUT_* / ROTOR_* — real motor capabilities for safety defaults.
 */

namespace m80106
{

    // ─────────────────────────────────────────────────────────────────────────────
    // Hardware constants — GO-M8010-6
    // ─────────────────────────────────────────────────────────────────────────────

    /// Output-to-rotor gear ratio (from the Unitree SDK / manufacturer README).
    /// Note: an independent teardown (SimplexityPD, 2025) counted 9-tooth sun /
    /// 47-tooth ring giving 1 + 47/9 ≈ 6.22.  We retain 6.33 to match the SDK
    /// encoding convention; adjust if your application requires higher accuracy.
    constexpr float GEAR_RATIO = 6.33f;

    /// Highest individually addressable motor ID on the RS-485 bus.
    /// ID 15 (BROADCAST_ID) sends to all motors simultaneously; no response is returned.
    constexpr uint8_t MAX_MOTOR_ID = 14;
    constexpr uint8_t BROADCAST_ID = 15;

    /// Temperature thresholds (°C).  Hardware protection activates at 90 °C.
    constexpr int8_t TEMP_WARNING_CELSIUS = 75;
    constexpr int8_t TEMP_PROTECTION_CELSIUS = 90;

    // ─────────────────────────────────────────────────────────────────────────────
    // Physical motor capabilities — GO-M8010-6
    // ─────────────────────────────────────────────────────────────────────────────
    // Source: Unitree GO-M8010-6 datasheet / product page.
    // These are the motor's real operating limits.  Use them for safety defaults,
    // torque budget calculations, and application-level clamps.

    /// Peak output-shaft torque [N·m].  Datasheet: ≈23.7 N·m.
    constexpr float OUTPUT_PEAK_TORQUE_NM = 23.7f;
    /// Continuous (rated) output-shaft torque [N·m].  Datasheet: ≈8.0 N·m.
    constexpr float OUTPUT_CONTINUOUS_TORQUE_NM = 8.0f;
    /// Derived rotor-side peak torque [N·m] = OUTPUT_PEAK_TORQUE_NM / GEAR_RATIO.
    constexpr float ROTOR_PEAK_TORQUE_NM = OUTPUT_PEAK_TORQUE_NM / GEAR_RATIO;
    /// Derived rotor-side continuous torque [N·m].
    constexpr float ROTOR_CONTINUOUS_TORQUE_NM = OUTPUT_CONTINUOUS_TORQUE_NM / GEAR_RATIO;

    /// Maximum output-shaft speed [rad/s].  Datasheet: ≈21 rad/s (≈200 RPM).
    constexpr float OUTPUT_MAX_SPEED_RADS = 21.0f;
    /// Derived rotor-side max speed [rad/s] = OUTPUT_MAX_SPEED_RADS * GEAR_RATIO.
    constexpr float ROTOR_MAX_SPEED_RADS = OUTPUT_MAX_SPEED_RADS * GEAR_RATIO;

    /// Rated supply voltage [V].
    constexpr float RATED_VOLTAGE_V = 24.0f;
    /// Maximum phase current [A] (approximate).
    constexpr float MAX_CURRENT_A = 16.0f;

    // ─────────────────────────────────────────────────────────────────────────────
    // Protocol encoding limits (Q8 / Q15 fixed-point maximums)
    // ─────────────────────────────────────────────────────────────────────────────
    // These are the largest values representable in the 17-byte SDK command
    // packet.  They are NOT the motor's physical capabilities.  Use them only
    // for clamping values before SDK serialisation (sendRecv).

    constexpr float PROTOCOL_MAX_TORQUE_NM = 127.99f;  ///< int16 Q8  → ±127.99 N·m
    constexpr float PROTOCOL_MAX_SPEED_RADS = 804.00f; ///< int16 custom scale → ±804 rad/s
    constexpr float PROTOCOL_MAX_POS_RAD = 411774.f;   ///< int32 custom scale → ±411 774 rad

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
    enum class MotorMode : uint8_t
    {
        BRAKE = 0,     ///< Locked / passive brake.
        FOC = 1,       ///< FOC closed-loop (torque / velocity / position).
        CALIBRATE = 2, ///< Encoder calibration.  Do not send other commands for ≥5 s.
    };

    /**
     * @brief Decoded error flags from MotorData::MError.
     */
    enum class MotorError : uint8_t
    {
        NONE = 0,
        OVERHEAT = 1,
        OVERCURRENT = 2,
        OVERVOLTAGE = 3,
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
        if (e >= 0 && e <= 4)
        {
            return static_cast<MotorError>(e);
        }
        return MotorError::NONE;
    }

    /// Human-readable error string.
    inline const char *errorString(MotorError e) noexcept
    {
        switch (e)
        {
        case MotorError::NONE:
            return "OK";
        case MotorError::OVERHEAT:
            return "Overheat";
        case MotorError::OVERCURRENT:
            return "Overcurrent";
        case MotorError::OVERVOLTAGE:
            return "Overvoltage";
        case MotorError::ENCODER_FAULT:
            return "Encoder Fault";
        }
        return "Unknown";
    }

    /// Human-readable mode string.
    inline const char *modeString(MotorMode m) noexcept
    {
        switch (m)
        {
        case MotorMode::BRAKE:
            return "BRAKE";
        case MotorMode::FOC:
            return "FOC";
        case MotorMode::CALIBRATE:
            return "CALIBRATE";
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
