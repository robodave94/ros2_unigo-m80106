#pragma once
/**
 * @file motor_config.hpp
 * @brief Per-motor configuration struct for GoM80106SetController.
 *
 * Defines GoMotorConfig — a plain aggregate describing one GO-M8010-6
 * actuator's operational limits and optional external-gearbox parameters.
 * The controller validates these at construction time.
 */

#include <cmath>
#include <cstdint>

#include "m80106_lib/motor_types.hpp"

namespace m80106
{

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

/// Default acceleration: 0 → OUTPUT_MAX_SPEED_RADS (21 rad/s) in 2 seconds.
constexpr float DEFAULT_ACCELERATION_RADS2 = OUTPUT_MAX_SPEED_RADS / 2.0f; // 10.5

/// Minimum allowed acceleration (motor-output-shaft).
constexpr float MIN_ACCELERATION_RADS2 = 0.5f;

/// Maximum allowed acceleration (motor-output-shaft).
/// Chosen to keep jerk tolerable — motor can physically exceed this but
/// doing so risks mechanical damage and uncontrolled motion.
constexpr float MAX_ACCELERATION_RADS2 = 200.0f;

/// ±1500 motor-output-shaft revolutions expressed in radians.
constexpr float MOTOR_POSITION_LIMIT_RAD =
    1500.0f * 2.0f * static_cast<float>(M_PI); // ≈ 9424.778 rad

/// Universal velocity limit — motor output shaft (= OUTPUT_MAX_SPEED_RADS).
constexpr float UNIVERSAL_VELOCITY_LIMIT_RADS = OUTPUT_MAX_SPEED_RADS; // 21 rad/s

// ─────────────────────────────────────────────────────────────────────────────
// GoMotorConfig
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Configuration for a single GO-M8010-6 actuator.
 *
 * All limit values refer to the **final output** — i.e. after the optional
 * external gearbox.  The controller converts them to motor-output-shaft and
 * rotor units internally.
 *
 * To create a config with only a motor ID (everything else at defaults):
 * @code
 *   GoMotorConfig cfg(3);          // motor ID 3, all defaults
 * @endcode
 */
struct GoMotorConfig
{
    /// Motor ID on the RS-485 bus (0–14).
    uint8_t motor_id;

    /// Maximum velocity at the final output [rad/s].
    /// Default: 21 rad/s (motor output shaft limit, no external gearbox).
    float velocity_limit_rads = UNIVERSAL_VELOCITY_LIMIT_RADS;

    /// Minimum allowed position at the motor output shaft [rad].
    /// Default: −9424.78 rad (−1500 revolutions).
    float position_min_rad = -MOTOR_POSITION_LIMIT_RAD;

    /// Maximum allowed position at the motor output shaft [rad].
    /// Default: +9424.78 rad (+1500 revolutions).
    float position_max_rad = MOTOR_POSITION_LIMIT_RAD;

    /// If false, runtime setters (setTargetPosition, etc.) are rejected.
    /// The motor is still polled for feedback at 100 Hz.
    bool modifiable = true;

    /// Whether an external gearbox is mounted on the motor output shaft.
    bool external_gearbox = false;

    /// Reduction ratio of the external gearbox.
    /// Must be > 0 when external_gearbox == true.
    /// Set to 0 (default) when no external gearbox is used.
    float external_gear_ratio = 0.0f;

    /// Efficiency of the external gearbox (0.0–1.0).
    /// Only applied when external_gearbox == true.
    /// Default: 0.9 (90 %).
    float external_gear_efficiency = 0.9f;

    /// Torque limit at the final output [N·m].
    /// Default: peak motor output torque (23.7 N·m, no external gearbox).
    float torque_limit_nm = OUTPUT_PEAK_TORQUE_NM;

    /// Acceleration profile at the final output [rad/s²].
    /// Controls how quickly the trapezoidal velocity profile ramps.
    /// Default: 10.5 rad/s² (0 → max speed in 2 seconds).
    float acceleration_rads2 = DEFAULT_ACCELERATION_RADS2;

    /// Construct with motor ID only — everything else uses defaults.
    explicit GoMotorConfig(uint8_t id) noexcept
        : motor_id(id)
    {
    }

    /// Full-parameter constructor.
    GoMotorConfig(uint8_t id,
                  float vel_limit,
                  float pos_min,
                  float pos_max,
                  bool modifiable_,
                  bool ext_gearbox,
                  float ext_gear_ratio,
                  float ext_gear_efficiency,
                  float torque_lim,
                  float accel) noexcept
        : motor_id(id)
        , velocity_limit_rads(vel_limit)
        , position_min_rad(pos_min)
        , position_max_rad(pos_max)
        , modifiable(modifiable_)
        , external_gearbox(ext_gearbox)
        , external_gear_ratio(ext_gear_ratio)
        , external_gear_efficiency(ext_gear_efficiency)
        , torque_limit_nm(torque_lim)
        , acceleration_rads2(accel)
    {
    }
};

/// Effective gear ratio for a config (1.0 if no external gearbox).
inline float effectiveGearRatio(const GoMotorConfig & cfg) noexcept
{
    return cfg.external_gearbox ? cfg.external_gear_ratio : 1.0f;
}

} // namespace m80106
