#pragma once

#include <memory>
#include <string>
#include <stdexcept>
#include <cmath>

// SDK headers — bundled in m80106_lib/sdk_include/
#include "unitreeMotor/unitreeMotor.h"
#include "m80106_lib/motor_types.hpp"

/**
 * @file control_modes.hpp
 * @brief Scalable control-mode abstraction for the Unitree GO-M8010-6 motor.
 *
 * ─── Design overview ────────────────────────────────────────────────────────
 *
 *  BaseControlMode  (abstract)
 *    │
 *    ├── Passthrough modes  (direct SDK map, no computation)
 *    │     BrakeMode          — locks shaft, all setpoints zero
 *    │     FOCPassthrough     — exposes raw SDK setpoints directly
 *    │     CalibrationMode    — sends encoder calibration command
 *    │
 *    ├── Custom wrapped modes  (use FOC + torque internally)
 *    │     PositionPD         — SDK K_P/K_W position PD controller
 *    │     VelocityPI         — proportional-integral on rotor speed
 *    │     DirectTorque       — raw torque, no gain
 *    │     ImpedanceControl   — spring-damper: T = k*(eq-Pos) - b*W
 *    │
 *    └── CRTP extension base
 *          CustomControlMode<Derived>  — inherit for user-defined modes
 *
 * ─── How to write a custom mode ─────────────────────────────────────────────
 *
 * @code
 *   struct GravityComp : m80106::CustomControlMode<GravityComp>
 *   {
 *     float gravity_torque_nm = 0.8f;
 *
 *     static m80106::SDKModeId  static_sdk_mode() { return m80106::SDKModeId::FOC; }
 *     static const char *       static_name()     { return "GravityComp"; }
 *
 *     MotorCmd onCompute(const MotorData &)
 *     {
 *       MotorCmd c = zeroCmd();
 *       c.T = gravity_torque_nm;
 *       return c;
 *     }
 *   };
 *
 *   GravityComp mode;
 *   mode.gravity_torque_nm = 1.2f;
 *   MotorCmd cmd = mode.compute(fb);
 *   cmd.id = 0;
 *   driver.sendRecv(cmd, fb);
 * @endcode
 *
 * ─── Typical usage with MotorDriver ─────────────────────────────────────────
 * @code
 *   m80106::PositionPD mode;
 *   mode.target_pos_rad = m80106::toRotorPos(1.5f);  // 1.5 rad output-side
 *   mode.kp             = m80106::toRotorKp(10.0f);
 *   mode.kd             = m80106::toRotorKd(1.0f);
 *
 *   MotorCmd cmd = mode.compute(last_fb);
 *   cmd.motorType = MotorType::GO_M8010_6;
 *   cmd.id        = 0;
 *   driver.sendRecv(cmd, last_fb);
 * @endcode
 */

namespace m80106
{

    // ─────────────────────────────────────────────────────────────────────────────
    // SDK mode numeric IDs — match MotorCmd::mode / MotorData::mode
    // ─────────────────────────────────────────────────────────────────────────────

    enum class SDKModeId : uint8_t
    {
        BRAKE = 0,
        FOC = 1,
        CALIBRATE = 2,
    };

    // ─────────────────────────────────────────────────────────────────────────────
    // Abstract base
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * @brief Abstract base class for all motor control modes.
     *
     * Contract:
     *   1. sdk_mode()  — returns the SDK hardware mode the motor must be in.
     *   2. name()      — returns a human-readable identifier string.
     *   3. compute(fb) — given the latest MotorData, returns a fully-populated
     *                    MotorCmd.  The caller (loop) must set cmd.motorType
     *                    and cmd.id before sending.
     *   4. reset()     — clears any internal state (integrators, etc.) when the
     *                    mode is activated or deactivated.  Optional to override.
     */
    class BaseControlMode
    {
    public:
        virtual ~BaseControlMode() = default;

        /// SDK hardware mode required by this control mode.
        virtual SDKModeId sdk_mode() const = 0;

        /// Human-readable mode identifier.
        virtual const char *name() const = 0;

        /**
         * @brief Compute next motor command given the latest feedback.
         *
         * The returned cmd has motorType and id set to zero-initialised values;
         * the calling loop must fill those fields before calling sendRecv().
         * All setpoint fields (mode, T, W, Pos, K_P, K_W) are populated here.
         *
         * @param fb  Most recent MotorData.  May have correct==false on startup
         *            or after a communication error; implementations should be
         *            tolerant of that.
         */
        virtual MotorCmd compute(const MotorData &fb) = 0;

        /// Reset internal state (integrators, history).  Call on mode switch.
        virtual void reset() {}

    protected:
        /**
         * @brief Return a zeroed MotorCmd pre-filled with the correct SDK mode.
         *
         * Derived classes call this inside compute() as a starting point to
         * avoid forgetting to set the mode field.
         */
        MotorCmd zeroCmd() const noexcept
        {
            MotorCmd c;
            c.motorType = MotorType::GO_M8010_6;
            c.id = 0; // caller sets
            c.mode = static_cast<uint16_t>(sdk_mode());
            c.T = 0.0f;
            c.W = 0.0f;
            c.Pos = 0.0f;
            c.K_P = 0.0f;
            c.K_W = 0.0f;
            return c;
        }
    };

    // ─────────────────────────────────────────────────────────────────────────────
    // Passthrough modes
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * @brief Brake mode — passively locks the motor shaft.
     *
     * All setpoints are zero.  No gains are applied; the motor holds its
     * current position through back-EMF braking only.
     */
    struct BrakeMode : BaseControlMode
    {
        SDKModeId sdk_mode() const override { return SDKModeId::BRAKE; }
        const char *name() const override { return "Brake"; }
        MotorCmd compute(const MotorData &) override { return zeroCmd(); }
    };

    /**
     * @brief Direct FOC passthrough — exposes all SDK setpoints verbatim.
     *
     * Set the public fields before each call to compute().  Values are clamped
     * to the hardware operating range.  Use toRotorKp() / toRotorKd() to
     * convert output-side gains to rotor-side.
     */
    struct FOCPassthrough : BaseControlMode
    {
        float T = 0.0f;   ///< Desired rotor torque   [N·m]   (protocol ±127.99, motor peak ±3.74)
        float W = 0.0f;   ///< Desired rotor speed    [rad/s] (protocol ±804, motor max ±133)
        float Pos = 0.0f; ///< Desired rotor position [rad]   (protocol ±411774)
        float K_P = 0.0f; ///< Position stiffness              0–25.599
        float K_W = 0.0f; ///< Velocity damping                0–25.599

        SDKModeId sdk_mode() const override { return SDKModeId::FOC; }
        const char *name() const override { return "FOC Passthrough"; }

        MotorCmd compute(const MotorData &) override
        {
            MotorCmd c = zeroCmd();
            c.T = clamp(T, -PROTOCOL_MAX_TORQUE_NM, PROTOCOL_MAX_TORQUE_NM);
            c.W = clamp(W, -PROTOCOL_MAX_SPEED_RADS, PROTOCOL_MAX_SPEED_RADS);
            c.Pos = clamp(Pos, -PROTOCOL_MAX_POS_RAD, PROTOCOL_MAX_POS_RAD);
            c.K_P = clamp(K_P, 0.0f, MAX_KP);
            c.K_W = clamp(K_W, 0.0f, MAX_KD);
            return c;
        }
    };

    /**
     * @brief Encoder calibration mode.
     *
     * Sends the one-shot calibration command.  After sending, do not issue any
     * other commands for at least 5 seconds.  The LED on the motor will blink
     * to indicate calibration is in progress.
     */
    struct CalibrationMode : BaseControlMode
    {
        SDKModeId sdk_mode() const override { return SDKModeId::CALIBRATE; }
        const char *name() const override { return "Calibration"; }
        MotorCmd compute(const MotorData &) override { return zeroCmd(); }
    };

    // ─────────────────────────────────────────────────────────────────────────────
    // Custom wrapped modes — all use FOC + torque as the actuation signal
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * @brief Position PD controller (rotor-side).
     *
     * Uses the SDK's built-in K_P / K_W coefficients so the position error and
     * velocity damping are computed on the motor microcontroller.  Gains are
     * rotor-side — use toRotorKp() / toRotorKd() for output-side design.
     *
     * Optionally adds a constant torque feed-forward (e.g. gravity compensation).
     *
     * @code
     *   m80106::PositionPD mode;
     *   mode.target_pos_rad = m80106::toRotorPos(1.5f);
     *   mode.kp             = m80106::toRotorKp(15.0f);
     *   mode.kd             = m80106::toRotorKd(1.5f);
     * @endcode
     */
    struct PositionPD : BaseControlMode
    {
        float target_pos_rad = 0.0f; ///< Target rotor position [rad]
        float kp = 1.0f;             ///< Rotor-side stiffness K_P  (0–25.599)
        float kd = 0.05f;            ///< Rotor-side damping   K_W  (0–25.599)
        float feedforward_T = 0.0f;  ///< Optional constant torque feed-forward [N·m]

        SDKModeId sdk_mode() const override { return SDKModeId::FOC; }
        const char *name() const override { return "Position PD"; }

        MotorCmd compute(const MotorData &) override
        {
            MotorCmd c = zeroCmd();
            c.Pos = clamp(target_pos_rad, -PROTOCOL_MAX_POS_RAD, PROTOCOL_MAX_POS_RAD);
            c.K_P = clamp(kp, 0.0f, MAX_KP);
            c.K_W = clamp(kd, 0.0f, MAX_KD);
            c.T = clamp(feedforward_T, -PROTOCOL_MAX_TORQUE_NM, PROTOCOL_MAX_TORQUE_NM);
            c.W = 0.0f;
            return c;
        }
    };

    /**
     * @brief Velocity PI controller — regulates rotor speed via torque output.
     *
     * Uses a proportional-integral controller:
     *   T = kp * e + ki * ∫e dt      where  e = target_speed - W
     *
     * The integral is clamped at ±integral_limit to prevent wind-up.
     *
     * @note Set dt_s to your control loop period before calling compute().
     *       Call reset() when switching to this mode.
     */
    struct VelocityPI : BaseControlMode
    {
        float target_speed_rads = 0.0f;              ///< Desired rotor speed [rad/s]
        float kp = 0.10f;                            ///< Proportional gain [N·m / (rad/s)]
        float ki = 0.01f;                            ///< Integral gain     [N·m / (rad/s·s)]
        float integral_limit = ROTOR_PEAK_TORQUE_NM; ///< Anti-windup clamp [N·m] (default: rotor peak)
        float dt_s = 0.001f;                         ///< Loop period [s] — caller must set
        /// Small inner-loop SDK damping for stability (leave at default).
        float inner_kd = 0.02f;

        SDKModeId sdk_mode() const override { return SDKModeId::FOC; }
        const char *name() const override { return "Velocity PI"; }

        void reset() override { integral_ = 0.0f; }

        MotorCmd compute(const MotorData &fb) override
        {
            const float error = target_speed_rads - fb.W;
            integral_ = clamp(integral_ + ki * error * dt_s,
                              -integral_limit, integral_limit);
            const float torque = kp * error + integral_;

            MotorCmd c = zeroCmd();
            c.T = clamp(torque, -PROTOCOL_MAX_TORQUE_NM, PROTOCOL_MAX_TORQUE_NM);
            c.K_W = clamp(inner_kd, 0.0f, MAX_KD);
            return c;
        }

    private:
        float integral_ = 0.0f;
    };

    /**
     * @brief Direct torque control — sends the requested torque with no gain.
     *
     * This is the most fundamental custom mode.  Use it as a building block
     * for outer-loop controllers (impedance, admittance, gravity compensation).
     */
    struct DirectTorque : BaseControlMode
    {
        float target_torque_nm = 0.0f; ///< Desired rotor torque [N·m] (protocol ±127.99, motor peak ±3.74)

        SDKModeId sdk_mode() const override { return SDKModeId::FOC; }
        const char *name() const override { return "Direct Torque"; }

        MotorCmd compute(const MotorData &) override
        {
            MotorCmd c = zeroCmd();
            c.T = clamp(target_torque_nm, -PROTOCOL_MAX_TORQUE_NM, PROTOCOL_MAX_TORQUE_NM);
            return c;
        }
    };

    /**
     * @brief Impedance control — virtual spring-damper behaviour.
     *
     * Implements:
     *   T = stiffness * (eq_pos_rad - Pos) - damping * W
     *
     * All quantities are rotor-side.  Use toRotorPos() for the equilibrium
     * position and toRotorKp()/toRotorKd() as a guide for gains.
     *
     * This is the recommended base for physical interaction tasks (human contact,
     * compliant joints, coupled oscillation suppression).  Override torque_limit
     * to enforce joint safety constraints.
     */
    struct ImpedanceControl : BaseControlMode
    {
        float eq_pos_rad = 0.0f;                   ///< Equilibrium rotor position [rad]
        float stiffness = 2.0f;                    ///< Virtual spring stiffness   [N·m/rad]
        float damping = 0.10f;                     ///< Virtual damper coefficient [N·m·s/rad]
        float torque_limit = ROTOR_PEAK_TORQUE_NM; ///< Safety torque saturation   [N·m] (default: rotor peak)

        SDKModeId sdk_mode() const override { return SDKModeId::FOC; }
        const char *name() const override { return "Impedance"; }

        MotorCmd compute(const MotorData &fb) override
        {
            const float torque = stiffness * (eq_pos_rad - fb.Pos) - damping * fb.W;

            MotorCmd c = zeroCmd();
            c.T = clamp(torque, -torque_limit, torque_limit);
            return c;
        }
    };

    // ─────────────────────────────────────────────────────────────────────────────
    // CRTP extension base
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * @brief CRTP helper for user-defined control modes.
     *
     * Inherit from this template and implement three members in Derived:
     *   - static SDKModeId  static_sdk_mode()       — hardware mode required
     *   - static const char * static_name()         — mode name string
     *   - MotorCmd onCompute(const MotorData & fb)  — compute the command
     *
     * zeroCmd() is available for convenience.  reset() has a no-op default.
     *
     * @code
     *   struct GravityComp : m80106::CustomControlMode<GravityComp>
     *   {
     *     float gravity_torque_nm = 0.8f;
     *
     *     static m80106::SDKModeId  static_sdk_mode() { return m80106::SDKModeId::FOC; }
     *     static const char *       static_name()     { return "GravityComp"; }
     *
     *     MotorCmd onCompute(const MotorData &)
     *     {
     *       MotorCmd c = zeroCmd();
     *       c.T = gravity_torque_nm;
     *       return c;
     *     }
     *   };
     * @endcode
     */
    template <typename Derived>
    struct CustomControlMode : BaseControlMode
    {
        SDKModeId sdk_mode() const override { return Derived::static_sdk_mode(); }
        const char *name() const override { return Derived::static_name(); }

        MotorCmd compute(const MotorData &fb) override
        {
            return static_cast<Derived *>(this)->onCompute(fb);
        }
    };

    // ─────────────────────────────────────────────────────────────────────────────
    // Factory
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * @brief Construct a built-in control mode by name string.
     *
     * Recognised names (case-sensitive):
     *   "brake"       → BrakeMode
     *   "foc"         → FOCPassthrough
     *   "calibration" → CalibrationMode
     *   "position_pd" → PositionPD
     *   "velocity_pi" → VelocityPI
     *   "torque"      → DirectTorque
     *   "impedance"   → ImpedanceControl
     *
     * @throws std::invalid_argument for unknown names.
     */
    inline std::unique_ptr<BaseControlMode> makeControlMode(const std::string &name)
    {
        if (name == "brake")
            return std::make_unique<BrakeMode>();
        if (name == "foc")
            return std::make_unique<FOCPassthrough>();
        if (name == "calibration")
            return std::make_unique<CalibrationMode>();
        if (name == "position_pd")
            return std::make_unique<PositionPD>();
        if (name == "velocity_pi")
            return std::make_unique<VelocityPI>();
        if (name == "torque")
            return std::make_unique<DirectTorque>();
        if (name == "impedance")
            return std::make_unique<ImpedanceControl>();
        throw std::invalid_argument("Unknown control mode: \"" + name + "\". "
                                                                        "Valid names: brake, foc, calibration, position_pd, velocity_pi, torque, impedance.");
    }

} // namespace m80106
