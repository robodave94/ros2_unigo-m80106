#pragma once
/**
 * @file go_m80106_set_controller.hpp
 * @brief High-level controller for a set of Unitree GO-M8010-6 actuators.
 *
 * GoM80106SetController discovers motors via USB PID:VID scanning, validates
 * them against a user-supplied list of GoMotorConfig structs, and runs a
 * background thread that communicates with every motor at 100 Hz in FOC mode
 * using a trapezoidal velocity profile.
 *
 * Thread safety:
 *   - Two mutexes protect shared state: targets_mutex_ (setpoints, offsets,
 *     flags) and feedback_mutex_ (last received MotorData).
 *   - Profile state is owned exclusively by the control thread.
 *   - User threads call setters/getters; they are safe from any thread.
 *
 * All user-facing quantities are in **final-output** units — i.e. after the
 * motor's internal 6.33:1 gearbox AND any external gearbox.  Rotational
 * values use radians.
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "m80106_lib/motor_config.hpp"
#include "m80106_lib/motor_driver.hpp"
#include "m80106_lib/motor_types.hpp"
#include "m80106_lib/multi_serial_go8_scanner.hpp"

namespace m80106
{

// ─────────────────────────────────────────────────────────────────────────────
// Internal per-motor state (not part of the public API)
// ─────────────────────────────────────────────────────────────────────────────
namespace detail
{

struct MotorState
{
    GoMotorConfig config;

    // ── Targets (written by user, read by control thread under targets_mutex_) ──
    float target_position_rad  = 0.0f; // motor-output-shaft, multi-turn [rad]
    float velocity_limit_rads;         // motor-output-shaft [rad/s]
    float acceleration_rads2;          // motor-output-shaft [rad/s²]
    float torque_limit_nm;             // motor-output-shaft [N·m]

    // ── Trajectory profile (owned by control thread — no lock needed) ──
    float profile_position_rad = 0.0f; // motor-output-shaft [rad]
    float profile_velocity_rads = 0.0f;// motor-output-shaft [rad/s]

    // ── Feedback (written by control thread under feedback_mutex_) ──
    MotorData last_feedback{};
    bool feedback_valid = false;

    // ── Position tracking (under targets_mutex_) ──
    float zero_offset_rad      = 0.0f; // motor-output-shaft offset
    bool  global_limit_exceeded = false;

    // ── Port association ──
    std::string port_path;

    explicit MotorState(const GoMotorConfig & cfg)
        : config(cfg)
        , velocity_limit_rads(cfg.velocity_limit_rads * effectiveGearRatio(cfg))
        , acceleration_rads2(cfg.acceleration_rads2 * effectiveGearRatio(cfg))
        , torque_limit_nm(cfg.torque_limit_nm / (effectiveGearRatio(cfg) *
            (cfg.external_gearbox ? cfg.external_gear_efficiency : 1.0f)))
    {
    }
};

} // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// GoM80106SetController
// ─────────────────────────────────────────────────────────────────────────────

class GoM80106SetController
{
public:
    /**
     * @brief Construct the controller.
     *
     * @param pidvid   USB PID:VID string for the RS-485 adapter(s).
     * @param configs  One GoMotorConfig per motor.  Motor IDs must be
     *                 unique, in [0, 14], and must exactly match the set of
     *                 motors discovered on the bus.
     *
     * @throws std::invalid_argument  Invalid motor ID, duplicate ID, or bad
     *                                 gearbox configuration.
     * @throws std::runtime_error      Scanned motor IDs do not match configs,
     *                                 or no ports found.
     */
    GoM80106SetController(const std::string & pidvid,
                          const std::vector<GoMotorConfig> & configs)
    {
        // ── 1. Validate configs ──────────────────────────────────────────
        validateConfigs(configs);

        // ── 2. Scan ports ────────────────────────────────────────────────
        auto scan = scanAllPorts(pidvid);
        if (scan.ports.empty())
        {
            throw std::runtime_error(
                "GoM80106SetController: No serial ports found for PID:VID '" +
                pidvid + "'");
        }

        // ── 3. Match config IDs to scanned IDs ──────────────────────────
        std::set<uint8_t> config_ids;
        for (const auto & c : configs) config_ids.insert(c.motor_id);

        std::set<uint8_t> scanned_ids;
        const auto all_motors = scan.allMotors();
        for (const auto & m : all_motors) scanned_ids.insert(m.id);

        if (config_ids != scanned_ids)
        {
            auto fmtSet = [](const std::set<uint8_t> & s) {
                std::string r = "{";
                for (auto it = s.begin(); it != s.end(); ++it) {
                    if (it != s.begin()) r += ", ";
                    r += std::to_string(static_cast<int>(*it));
                }
                return r + "}";
            };
            throw std::runtime_error(
                "GoM80106SetController: Motor ID mismatch.\n"
                "  Expected (configs): " + fmtSet(config_ids) + "\n"
                "  Found (scan):       " + fmtSet(scanned_ids));
        }

        // ── 4. Build port→motor mapping ──────────────────────────────────
        // Map from motor_id → port_path
        std::map<uint8_t, std::string> id_to_port;
        for (const auto & m : all_motors) id_to_port[m.id] = m.port;

        // ── 5. Create MotorDrivers (one per unique port) ────────────────
        std::set<std::string> unique_ports;
        for (const auto & kv : id_to_port) unique_ports.insert(kv.second);

        for (const auto & port : unique_ports)
        {
            drivers_[port] = std::make_unique<MotorDriver>(port);
        }

        // ── 6. Initialise per-motor state ────────────────────────────────
        for (const auto & cfg : configs)
        {
            auto state = detail::MotorState(cfg);
            state.port_path = id_to_port[cfg.motor_id];

            // Read initial position via a brake command
            MotorData fb{};
            auto * drv = drivers_[state.port_path].get();
            if (drv->brake(cfg.motor_id, fb) && fb.correct)
            {
                float motor_output_pos = toOutputPos(fb.Pos);
                state.profile_position_rad = motor_output_pos;
                state.target_position_rad  = motor_output_pos;
                state.last_feedback        = fb;
                state.feedback_valid       = true;
            }

            motors_.emplace(cfg.motor_id, std::move(state));
        }

        // ── 7. Start control thread ─────────────────────────────────────
        running_.store(true);
        control_thread_ = std::thread(&GoM80106SetController::controlLoop, this);
    }

    ~GoM80106SetController()
    {
        running_.store(false);
        if (control_thread_.joinable()) control_thread_.join();

        // Final brake pass
        for (auto & [id, state] : motors_)
        {
            auto it = drivers_.find(state.port_path);
            if (it != drivers_.end())
            {
                MotorData fb;
                it->second->brake(id, fb);
            }
        }
    }

    // Non-copyable, non-movable
    GoM80106SetController(const GoM80106SetController &) = delete;
    GoM80106SetController & operator=(const GoM80106SetController &) = delete;

    // ─────────────────────────────────────────────────────────────────────
    // Setters — single motor
    // ─────────────────────────────────────────────────────────────────────

    /**
     * @brief Set target position (multi-turn, radians, final output).
     *
     * Positions outside the ±1500-revolution motor-output-shaft limit
     * (divided by external gear ratio) are rejected with a warning.
     */
    bool setTargetPosition(uint8_t id, float position_rad)
    {
        std::lock_guard<std::mutex> lock(targets_mutex_);
        auto * s = findMotor(id);
        if (!s) { warnInvalidId(id); return false; }
        if (!s->config.modifiable) { warnNotModifiable(id); return false; }
        if (s->global_limit_exceeded) { warnGlobalLimit(id); return false; }

        float eff = effectiveGearRatio(s->config);
        float motor_pos = position_rad * eff;

        if (motor_pos < s->config.position_min_rad ||
            motor_pos > s->config.position_max_rad)
        {
            std::fprintf(stderr,
                "[GoM80106SetController] WARNING: setTargetPosition(%d, %.3f rad) "
                "exceeds motor position limits [%.1f, %.1f] rad (motor output). "
                "Command rejected.\n",
                id, position_rad,
                s->config.position_min_rad / eff,
                s->config.position_max_rad / eff);
            return false;
        }

        s->target_position_rad = motor_pos;
        return true;
    }

    /**
     * @brief Set velocity limit (rad/s, final output).
     *
     * Clamped to UNIVERSAL_VELOCITY_LIMIT_RADS / effective_gear_ratio.
     */
    bool setVelocityLimit(uint8_t id, float velocity_rads)
    {
        std::lock_guard<std::mutex> lock(targets_mutex_);
        auto * s = findMotor(id);
        if (!s) { warnInvalidId(id); return false; }
        if (!s->config.modifiable) { warnNotModifiable(id); return false; }

        float eff = effectiveGearRatio(s->config);
        float max_final = UNIVERSAL_VELOCITY_LIMIT_RADS / eff;

        if (velocity_rads <= 0.0f || velocity_rads > max_final)
        {
            std::fprintf(stderr,
                "[GoM80106SetController] WARNING: setVelocityLimit(%d, %.3f rad/s) "
                "out of valid range (0, %.3f]. Command rejected.\n",
                id, velocity_rads, max_final);
            return false;
        }

        s->velocity_limit_rads = velocity_rads * eff; // store as motor-output
        return true;
    }

    /**
     * @brief Set acceleration profile (rad/s², final output).
     *
     * Clamped to [MIN_ACCELERATION_RADS2, MAX_ACCELERATION_RADS2] at the
     * motor output shaft (accounting for external gearbox).
     */
    bool setAccelerationProfile(uint8_t id, float accel_rads2)
    {
        std::lock_guard<std::mutex> lock(targets_mutex_);
        auto * s = findMotor(id);
        if (!s) { warnInvalidId(id); return false; }
        if (!s->config.modifiable) { warnNotModifiable(id); return false; }

        float eff = effectiveGearRatio(s->config);
        float motor_accel = accel_rads2 * eff;

        if (motor_accel < MIN_ACCELERATION_RADS2 ||
            motor_accel > MAX_ACCELERATION_RADS2)
        {
            std::fprintf(stderr,
                "[GoM80106SetController] WARNING: setAccelerationProfile(%d, %.3f rad/s²) "
                "out of valid range [%.3f, %.3f] (final output). Command rejected.\n",
                id, accel_rads2,
                MIN_ACCELERATION_RADS2 / eff,
                MAX_ACCELERATION_RADS2 / eff);
            return false;
        }

        s->acceleration_rads2 = motor_accel;
        return true;
    }

    /**
     * @brief Set torque limit (N·m, final output).
     *
     * Limits the commanded torque at the motor output shaft (accounting for
     * external gearbox ratio and efficiency).
     */
    bool setTorqueLimit(uint8_t id, float torque_nm)
    {
        std::lock_guard<std::mutex> lock(targets_mutex_);
        auto * s = findMotor(id);
        if (!s) { warnInvalidId(id); return false; }
        if (!s->config.modifiable) { warnNotModifiable(id); return false; }

        float eff  = effectiveGearRatio(s->config);
        float efcy = s->config.external_gearbox
                         ? s->config.external_gear_efficiency
                         : 1.0f;

        // Convert final-output torque to motor-output-shaft torque
        float motor_torque = torque_nm / (eff * efcy);

        if (motor_torque <= 0.0f || motor_torque > OUTPUT_PEAK_TORQUE_NM)
        {
            float max_final = OUTPUT_PEAK_TORQUE_NM * eff * efcy;
            std::fprintf(stderr,
                "[GoM80106SetController] WARNING: setTorqueLimit(%d, %.3f N·m) "
                "out of valid range (0, %.3f]. Command rejected.\n",
                id, torque_nm, max_final);
            return false;
        }

        s->torque_limit_nm = motor_torque;
        return true;
    }

    /**
     * @brief Zero the position of motor @p id.
     *
     * The motor's current position becomes the new zero reference.
     * getPosition() will return ~0 immediately; getGlobalPosition() is
     * unaffected.  The global ±1500-revolution limit still tracks from
     * the true motor origin.
     */
    bool zeroPosition(uint8_t id)
    {
        std::lock_guard<std::mutex> tlock(targets_mutex_);
        std::lock_guard<std::mutex> flock(feedback_mutex_);
        auto * s = findMotor(id);
        if (!s) { warnInvalidId(id); return false; }

        if (s->feedback_valid)
        {
            s->zero_offset_rad = toOutputPos(s->last_feedback.Pos);
        }
        return true;
    }

    // ─────────────────────────────────────────────────────────────────────
    // Setters — multi-motor
    // ─────────────────────────────────────────────────────────────────────

    std::map<uint8_t, bool> setTargetPosition(
        const std::map<uint8_t, float> & targets)
    {
        std::map<uint8_t, bool> results;
        for (const auto & [id, val] : targets)
            results[id] = setTargetPosition(id, val);
        return results;
    }

    std::map<uint8_t, bool> setVelocityLimit(
        const std::map<uint8_t, float> & limits)
    {
        std::map<uint8_t, bool> results;
        for (const auto & [id, val] : limits)
            results[id] = setVelocityLimit(id, val);
        return results;
    }

    std::map<uint8_t, bool> setAccelerationProfile(
        const std::map<uint8_t, float> & profiles)
    {
        std::map<uint8_t, bool> results;
        for (const auto & [id, val] : profiles)
            results[id] = setAccelerationProfile(id, val);
        return results;
    }

    std::map<uint8_t, bool> setTorqueLimit(
        const std::map<uint8_t, float> & limits)
    {
        std::map<uint8_t, bool> results;
        for (const auto & [id, val] : limits)
            results[id] = setTorqueLimit(id, val);
        return results;
    }

    std::map<uint8_t, bool> zeroPosition(const std::vector<uint8_t> & ids)
    {
        std::map<uint8_t, bool> results;
        for (uint8_t id : ids)
            results[id] = zeroPosition(id);
        return results;
    }

    // ─────────────────────────────────────────────────────────────────────
    // Getters — single motor
    // ─────────────────────────────────────────────────────────────────────

    /// Zeroed position (rad, final output).
    float getPosition(uint8_t id) const
    {
        std::lock_guard<std::mutex> flock(feedback_mutex_);
        std::lock_guard<std::mutex> tlock(targets_mutex_);
        const auto * s = findMotor(id);
        if (!s || !s->feedback_valid) return 0.0f;
        float motor_out = toOutputPos(s->last_feedback.Pos);
        float eff = effectiveGearRatio(s->config);
        return (motor_out - s->zero_offset_rad) / eff;
    }

    /// Global (un-zeroed) position (rad, final output).
    float getGlobalPosition(uint8_t id) const
    {
        std::lock_guard<std::mutex> flock(feedback_mutex_);
        const auto * s = findMotor(id);
        if (!s || !s->feedback_valid) return 0.0f;
        float motor_out = toOutputPos(s->last_feedback.Pos);
        float eff = effectiveGearRatio(s->config);
        return motor_out / eff;
    }

    /// Velocity (rad/s, final output).
    float getVelocity(uint8_t id) const
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        const auto * s = findMotor(id);
        if (!s || !s->feedback_valid) return 0.0f;
        float motor_out_speed = toOutputSpeed(s->last_feedback.W);
        float eff = effectiveGearRatio(s->config);
        return motor_out_speed / eff;
    }

    /// Torque (N·m, final output).
    float getTorque(uint8_t id) const
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        const auto * s = findMotor(id);
        if (!s || !s->feedback_valid) return 0.0f;
        float motor_out_torque = s->last_feedback.T * GEAR_RATIO; // rotor→output
        float eff  = effectiveGearRatio(s->config);
        float efcy = s->config.external_gearbox
                         ? s->config.external_gear_efficiency
                         : 1.0f;
        return motor_out_torque * eff * efcy;
    }

    /// Temperature (°C).
    int getTemperature(uint8_t id) const
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        const auto * s = findMotor(id);
        if (!s || !s->feedback_valid) return 0;
        return s->last_feedback.Temp;
    }

    /// Error code (0=OK, 1=OVERHEAT, 2=OVERCURRENT, 3=OVERVOLTAGE, 4=ENCODER_FAULT).
    int getErrorCode(uint8_t id) const
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        const auto * s = findMotor(id);
        if (!s || !s->feedback_valid) return 0;
        return s->last_feedback.MError;
    }

    /// Motor operating mode (0=BRAKE, 1=FOC, 2=CALIBRATE).
    uint8_t getMode(uint8_t id) const
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        const auto * s = findMotor(id);
        if (!s || !s->feedback_valid) return 0;
        return s->last_feedback.mode;
    }

    /// CRC + ID integrity check from last packet.
    bool getCorrect(uint8_t id) const
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        const auto * s = findMotor(id);
        if (!s || !s->feedback_valid) return false;
        return s->last_feedback.correct;
    }

    /// Foot-force ADC reading (0–4095).
    int getFootForce(uint8_t id) const
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        const auto * s = findMotor(id);
        if (!s || !s->feedback_valid) return 0;
        return s->last_feedback.footForce;
    }

    // ─────────────────────────────────────────────────────────────────────
    // Getters — multi-motor
    // ─────────────────────────────────────────────────────────────────────

    std::map<uint8_t, float> getPosition(const std::vector<uint8_t> & ids) const
    {
        std::map<uint8_t, float> r;
        for (uint8_t id : ids) r[id] = getPosition(id);
        return r;
    }

    std::map<uint8_t, float> getGlobalPosition(const std::vector<uint8_t> & ids) const
    {
        std::map<uint8_t, float> r;
        for (uint8_t id : ids) r[id] = getGlobalPosition(id);
        return r;
    }

    std::map<uint8_t, float> getVelocity(const std::vector<uint8_t> & ids) const
    {
        std::map<uint8_t, float> r;
        for (uint8_t id : ids) r[id] = getVelocity(id);
        return r;
    }

    std::map<uint8_t, float> getTorque(const std::vector<uint8_t> & ids) const
    {
        std::map<uint8_t, float> r;
        for (uint8_t id : ids) r[id] = getTorque(id);
        return r;
    }

    std::map<uint8_t, int> getTemperature(const std::vector<uint8_t> & ids) const
    {
        std::map<uint8_t, int> r;
        for (uint8_t id : ids) r[id] = getTemperature(id);
        return r;
    }

    std::map<uint8_t, int> getErrorCode(const std::vector<uint8_t> & ids) const
    {
        std::map<uint8_t, int> r;
        for (uint8_t id : ids) r[id] = getErrorCode(id);
        return r;
    }

    std::map<uint8_t, uint8_t> getMode(const std::vector<uint8_t> & ids) const
    {
        std::map<uint8_t, uint8_t> r;
        for (uint8_t id : ids) r[id] = getMode(id);
        return r;
    }

    std::map<uint8_t, bool> getCorrect(const std::vector<uint8_t> & ids) const
    {
        std::map<uint8_t, bool> r;
        for (uint8_t id : ids) r[id] = getCorrect(id);
        return r;
    }

    std::map<uint8_t, int> getFootForce(const std::vector<uint8_t> & ids) const
    {
        std::map<uint8_t, int> r;
        for (uint8_t id : ids) r[id] = getFootForce(id);
        return r;
    }

private:
    // ─────────────────────────────────────────────────────────────────────
    // Synchronisation
    // ─────────────────────────────────────────────────────────────────────

    std::thread control_thread_;
    std::atomic<bool> running_{false};

    /// Protects target values, zero_offset, global_limit_exceeded.
    mutable std::mutex targets_mutex_;

    /// Protects last_feedback, feedback_valid.
    mutable std::mutex feedback_mutex_;

    // ─────────────────────────────────────────────────────────────────────
    // State
    // ─────────────────────────────────────────────────────────────────────

    std::map<uint8_t, detail::MotorState> motors_;
    std::map<std::string, std::unique_ptr<MotorDriver>> drivers_;

    // ─────────────────────────────────────────────────────────────────────
    // Validation (called from constructor)
    // ─────────────────────────────────────────────────────────────────────

    static void validateConfigs(const std::vector<GoMotorConfig> & configs)
    {
        if (configs.empty())
        {
            throw std::invalid_argument(
                "GoM80106SetController: configs vector is empty.");
        }

        std::set<uint8_t> seen;
        for (const auto & c : configs)
        {
            // Range check
            if (c.motor_id > MAX_MOTOR_ID)
            {
                throw std::invalid_argument(
                    "GoM80106SetController: motor_id " +
                    std::to_string(static_cast<int>(c.motor_id)) +
                    " is outside the valid range [0, " +
                    std::to_string(static_cast<int>(MAX_MOTOR_ID)) + "].");
            }

            // Duplicate check
            if (!seen.insert(c.motor_id).second)
            {
                throw std::invalid_argument(
                    "GoM80106SetController: duplicate motor_id " +
                    std::to_string(static_cast<int>(c.motor_id)) + ".");
            }

            // Gearbox consistency
            if (c.external_gearbox && c.external_gear_ratio <= 0.0f)
            {
                throw std::invalid_argument(
                    "GoM80106SetController: motor_id " +
                    std::to_string(static_cast<int>(c.motor_id)) +
                    " has external_gearbox=true but external_gear_ratio <= 0.");
            }

            if (c.external_gearbox &&
                (c.external_gear_efficiency <= 0.0f ||
                 c.external_gear_efficiency > 1.0f))
            {
                throw std::invalid_argument(
                    "GoM80106SetController: motor_id " +
                    std::to_string(static_cast<int>(c.motor_id)) +
                    " has invalid external_gear_efficiency (must be (0, 1]).");
            }

            // Velocity limit
            float eff = effectiveGearRatio(c);
            float motor_vel = c.velocity_limit_rads * eff;
            if (motor_vel <= 0.0f || motor_vel > UNIVERSAL_VELOCITY_LIMIT_RADS)
            {
                throw std::invalid_argument(
                    "GoM80106SetController: motor_id " +
                    std::to_string(static_cast<int>(c.motor_id)) +
                    " velocity_limit_rads " +
                    std::to_string(c.velocity_limit_rads) +
                    " exceeds motor capability when converted to motor-output.");
            }

            // Acceleration
            float motor_accel = c.acceleration_rads2 * eff;
            if (motor_accel < MIN_ACCELERATION_RADS2 ||
                motor_accel > MAX_ACCELERATION_RADS2)
            {
                throw std::invalid_argument(
                    "GoM80106SetController: motor_id " +
                    std::to_string(static_cast<int>(c.motor_id)) +
                    " acceleration_rads2 " +
                    std::to_string(c.acceleration_rads2) +
                    " is outside valid range [" +
                    std::to_string(MIN_ACCELERATION_RADS2 / eff) + ", " +
                    std::to_string(MAX_ACCELERATION_RADS2 / eff) + "].");
            }
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Lookup helpers
    // ─────────────────────────────────────────────────────────────────────

    detail::MotorState * findMotor(uint8_t id)
    {
        auto it = motors_.find(id);
        return (it != motors_.end()) ? &it->second : nullptr;
    }

    const detail::MotorState * findMotor(uint8_t id) const
    {
        auto it = motors_.find(id);
        return (it != motors_.end()) ? &it->second : nullptr;
    }

    // ─────────────────────────────────────────────────────────────────────
    // Warning helpers
    // ─────────────────────────────────────────────────────────────────────

    static void warnInvalidId(uint8_t id)
    {
        std::fprintf(stderr,
            "[GoM80106SetController] WARNING: motor ID %d not found in "
            "controller.\n", static_cast<int>(id));
    }

    static void warnNotModifiable(uint8_t id)
    {
        std::fprintf(stderr,
            "[GoM80106SetController] WARNING: motor ID %d is not modifiable.\n",
            static_cast<int>(id));
    }

    static void warnGlobalLimit(uint8_t id)
    {
        std::fprintf(stderr,
            "[GoM80106SetController] WARNING: motor ID %d has exceeded its "
            "global position limit (±1500 revolutions). Power cycle required. "
            "Command rejected.\n", static_cast<int>(id));
    }

    // ─────────────────────────────────────────────────────────────────────
    // Trapezoidal velocity profile update
    // ─────────────────────────────────────────────────────────────────────

    static void updateTrajectory(detail::MotorState & state, float dt)
    {
        float error = state.target_position_rad - state.profile_position_rad;
        float abs_error = std::fabs(error);
        float accel = state.acceleration_rads2;
        float vel_limit = state.velocity_limit_rads;
        float cur_vel = state.profile_velocity_rads;

        // Stopping distance at current speed
        float stopping_dist =
            (cur_vel * cur_vel) / (2.0f * accel);

        // Determine desired velocity
        float desired_vel;
        if (abs_error < 1e-5f && std::fabs(cur_vel) < 1e-4f)
        {
            // At target and essentially stopped
            state.profile_velocity_rads = 0.0f;
            state.profile_position_rad  = state.target_position_rad;
            return;
        }

        float direction = (error > 0.0f) ? 1.0f : -1.0f;

        // Check if we need to decelerate
        bool moving_toward_target =
            (cur_vel * direction) > 0.0f || std::fabs(cur_vel) < 1e-4f;

        if (moving_toward_target && abs_error > stopping_dist)
        {
            // Accelerate (or maintain) toward target
            desired_vel = direction * vel_limit;
        }
        else
        {
            // Decelerate toward zero velocity at the target
            // v² = 2 * a * d  →  v = sqrt(2 * a * d) in the direction of error
            float decel_vel = std::sqrt(2.0f * accel * abs_error);
            desired_vel = direction * std::min(decel_vel, vel_limit);
        }

        // Apply acceleration limit
        float dv = desired_vel - cur_vel;
        float max_dv = accel * dt;
        dv = clamp(dv, -max_dv, max_dv);

        state.profile_velocity_rads = cur_vel + dv;

        // Clamp velocity to limit
        state.profile_velocity_rads = clamp(
            state.profile_velocity_rads, -vel_limit, vel_limit);

        // Integrate position
        state.profile_position_rad += state.profile_velocity_rads * dt;
    }

    // ─────────────────────────────────────────────────────────────────────
    // FOC command builder
    // ─────────────────────────────────────────────────────────────────────

    /// Build an FOC MotorCmd from the current profile state.
    /// Position PD with velocity feedforward.
    static MotorCmd buildFOCCommand(const detail::MotorState & state)
    {
        MotorCmd cmd;
        cmd.motorType = MotorType::GO_M8010_6;
        cmd.id        = state.config.motor_id;
        cmd.mode      = toSDKMode(MotorMode::FOC);

        // Convert motor-output-shaft → rotor
        float rotor_pos = toRotorPos(state.profile_position_rad);
        float rotor_vel = toRotorSpeed(state.profile_velocity_rads);

        // Position PD gains (output-side equivalent ≈ 15 / 1.5)
        float K_P = toRotorKp(15.0f);
        float K_W = toRotorKd(1.5f);

        // Minimum damping floor to prevent uncontrolled motion
        constexpr float MIN_KW = 0.0125f; // ≈ toRotorKd(0.5)
        if (K_W < MIN_KW) K_W = MIN_KW;

        cmd.T   = 0.0f; // Torque handled via PD gains
        cmd.W   = clamp(rotor_vel,
                         -PROTOCOL_MAX_SPEED_RADS, PROTOCOL_MAX_SPEED_RADS);
        cmd.Pos = clamp(rotor_pos,
                         -PROTOCOL_MAX_POS_RAD, PROTOCOL_MAX_POS_RAD);
        cmd.K_P = clamp(K_P, 0.0f, MAX_KP);
        cmd.K_W = clamp(K_W, 0.0f, MAX_KD);

        return cmd;
    }

    // ─────────────────────────────────────────────────────────────────────
    // 100 Hz control loop
    // ─────────────────────────────────────────────────────────────────────

    void controlLoop()
    {
        using clock = std::chrono::steady_clock;
        constexpr auto period = std::chrono::microseconds(10000); // 100 Hz
        constexpr float dt = 0.01f; // seconds

        while (running_.load())
        {
            auto cycle_start = clock::now();

            // Process each port's motors sequentially (RS-485 is half-duplex)
            for (auto & [port, driver] : drivers_)
            {
                for (auto & [id, state] : motors_)
                {
                    if (state.port_path != port) continue;

                    // 1. Copy targets under lock
                    float target_pos, vel_limit, accel, torque_lim;
                    bool limit_exceeded;
                    {
                        std::lock_guard<std::mutex> lock(targets_mutex_);
                        target_pos     = state.target_position_rad;
                        vel_limit      = state.velocity_limit_rads;
                        accel          = state.acceleration_rads2;
                        torque_lim     = state.torque_limit_nm;
                        limit_exceeded = state.global_limit_exceeded;
                    }

                    // 2. If global limit exceeded, send brake
                    if (limit_exceeded)
                    {
                        MotorData fb;
                        driver->brake(id, fb);
                        if (fb.correct)
                        {
                            std::lock_guard<std::mutex> lock(feedback_mutex_);
                            state.last_feedback = fb;
                            state.feedback_valid = true;
                        }
                        continue;
                    }

                    // Apply copied targets to profile state
                    // (profile state is owned by this thread — no lock)
                    state.target_position_rad  = target_pos;
                    state.velocity_limit_rads  = vel_limit;
                    state.acceleration_rads2   = accel;
                    state.torque_limit_nm      = torque_lim;

                    // 3. Update trapezoidal trajectory
                    updateTrajectory(state, dt);

                    // 4. Build and send FOC command
                    MotorCmd cmd = buildFOCCommand(state);

                    // Apply torque limit as a clamp on the SDK gains
                    // The PD controller internally limits torque via K_P/K_W
                    // We also cap T (though it's 0 for position tracking)
                    float rotor_torque_limit = torque_lim / GEAR_RATIO;
                    cmd.T = clamp(cmd.T,
                                  -rotor_torque_limit, rotor_torque_limit);

                    MotorData fb;
                    driver->sendRecv(cmd, fb);

                    // 5. Store feedback
                    if (fb.correct)
                    {
                        std::lock_guard<std::mutex> flock(feedback_mutex_);
                        state.last_feedback  = fb;
                        state.feedback_valid = true;
                    }

                    // 6. Check global position limit
                    if (fb.correct)
                    {
                        float motor_out_pos = toOutputPos(fb.Pos);
                        if (std::fabs(motor_out_pos) > MOTOR_POSITION_LIMIT_RAD)
                        {
                            std::lock_guard<std::mutex> lock(targets_mutex_);
                            state.global_limit_exceeded = true;
                            std::fprintf(stderr,
                                "[GoM80106SetController] ERROR: Motor ID %d "
                                "exceeded ±1500 revolution limit (pos=%.1f rad, "
                                "%.1f revs). Motor locked. Power cycle required.\n",
                                static_cast<int>(id),
                                motor_out_pos,
                                motor_out_pos / (2.0f * static_cast<float>(M_PI)));
                        }
                    }
                }
            }

            // Sleep until next cycle
            auto elapsed = clock::now() - cycle_start;
            if (elapsed < period)
            {
                std::this_thread::sleep_until(cycle_start + period);
            }
        }
    }
};

} // namespace m80106
