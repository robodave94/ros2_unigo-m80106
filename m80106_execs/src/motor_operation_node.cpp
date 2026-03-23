/**
 * @file motor_operation_node.cpp
 * @brief Preprogrammed motion-verification routine for the Unitree GO-M8010-6.
 *
 * Scans all RS-485 adapters matching the given PID:VID, locates the target
 * motor, then executes a short (~30 s) motion sequence that exercises every
 * major control mode:
 *
 *   [1/5] BRAKE    — read initial position and telemetry.
 *   [2/5] VELOCITY — +5 rad/s output for 3 s, then -5 rad/s for 3 s.
 *   [3/5] POSITION — PD: move to +0.5 rad (output), then -0.5 rad,
 *                    then return to start position.
 *   [4/5] TORQUE   — direct torque: +1.5 N·m for 2 s, then -1.5 N·m for 2 s.
 *   [5/5] BRAKE    — safe final state.
 *
 * Total motion time: ~30 s (well under the 60 s target).
 *
 * Control-mode conventions (all quantities rotor-side in the SDK):
 *   Velocity: K_P=0, K_W=damping, W=desired_rotor_speed, T=0, Pos=0
 *   Position: K_P=stiffness, K_W=damping, Pos=target_rotor_pos, W=0, T=0
 *   Torque:   K_P=0, K_W≥floor, T=desired_torque, W=0, Pos=0
 *
 * A minimum velocity-damping floor (0.5 output-side) is applied to every
 * FOC command to limit jerk and prevent uncontrolled acceleration.
 *
 * Node parameters:
 *   pidvid    (string, default "0403:6011") — USB PID:VID of the RS-485 adapter.
 *   motor_id  (int,    default 0)           — Motor ID to operate (0–14).
 *
 * Usage:
 *   ros2 run m80106_execs motor_operation --ros-args -p motor_id:=0
 *   ros2 run m80106_execs motor_operation --ros-args -p motor_id:=2 -p pidvid:=0403:6011
 */

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "m80106_lib/motor_driver.hpp"
#include "m80106_lib/motor_types.hpp"

#include "m80106_execs/multi_serial_go8_scanner.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Minimum velocity-damping floor (output-side 0.5 → rotor-side ≈ 0.0125).
/// Applied to every FOC command to prevent uncontrolled acceleration and
/// reduce jerk during setpoint transitions.
static const float MIN_DAMPING_KW = m80106::toRotorKd(0.5f);

/// Build a FOC MotorCmd (all fields rotor-side, clamped to hardware limits).
/// K_W is floored to MIN_DAMPING_KW so no FOC command is ever fully undamped.
static MotorCmd makeFOCCmd(uint8_t id,
                           float T, float W, float Pos,
                           float K_P, float K_W)
{
    MotorCmd cmd;
    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id        = id;
    cmd.mode      = m80106::toSDKMode(m80106::MotorMode::FOC);
    cmd.T         = m80106::clamp(T,   -m80106::MAX_TORQUE_NM,  m80106::MAX_TORQUE_NM);
    cmd.W         = m80106::clamp(W,   -m80106::MAX_SPEED_RADS, m80106::MAX_SPEED_RADS);
    cmd.Pos       = m80106::clamp(Pos, -m80106::MAX_POS_RAD,    m80106::MAX_POS_RAD);
    cmd.K_P       = m80106::clamp(K_P,  0.0f, m80106::MAX_KP);
    cmd.K_W       = std::max(m80106::clamp(K_W, 0.0f, m80106::MAX_KD), MIN_DAMPING_KW);
    return cmd;
}

/// Send @p cmd at 100 Hz for @p duration_ms milliseconds.
/// Updates @p last_fb with the most recent valid feedback received.
static void runFor(m80106::MotorDriver & driver,
                   const MotorCmd & cmd,
                   int duration_ms,
                   MotorData & last_fb)
{
    constexpr int rate_hz   = 100;
    constexpr int period_us = 1000000 / rate_hz;
    const int     steps     = (duration_ms * rate_hz) / 1000;

    for (int i = 0; i < steps && rclcpp::ok(); ++i) {
        MotorCmd c = cmd;
        MotorData fb;
        if (driver.sendRecv(c, fb) && fb.correct) {
            last_fb = fb;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    }
}

/// Issue brake commands at 100 Hz for @p duration_ms milliseconds.
static void brakeFor(m80106::MotorDriver & driver, uint8_t id,
                     int duration_ms, MotorData & last_fb)
{
    constexpr int rate_hz   = 100;
    constexpr int period_us = 1000000 / rate_hz;
    const int     steps     = (duration_ms * rate_hz) / 1000;

    for (int i = 0; i < steps && rclcpp::ok(); ++i) {
        MotorData fb;
        if (driver.brake(id, fb) && fb.correct) {
            last_fb = fb;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    }
}

/// Log a one-line telemetry snapshot (output-side values).
static void logState(rclcpp::Logger logger, const MotorData & fb)
{
    if (!fb.correct) {
        RCLCPP_WARN(logger, "  (no valid feedback)");
        return;
    }
    RCLCPP_INFO(logger,
        "  ID=%-2d  Mode=%-9s  Pos=%+.3f rad(out)  "
        "Speed=%+.2f rad/s(out)  Torque=%+.2f N·m  Temp=%d°C",
        static_cast<int>(fb.motor_id),
        m80106::modeString(static_cast<m80106::MotorMode>(fb.mode)),
        m80106::toOutputPos(fb.Pos),
        m80106::toOutputSpeed(fb.W),
        fb.T,
        fb.Temp);
}

/// Check feedback for hardware errors. Returns true if an error was detected.
static bool checkError(rclcpp::Logger logger, const MotorData & fb)
{
    if (!fb.correct) return false;
    auto err = m80106::toMotorError(fb.MError);
    if (err != m80106::MotorError::NONE) {
        RCLCPP_ERROR(logger, "  *** HARDWARE ERROR: %s (code %d) ***",
                     m80106::errorString(err), fb.MError);
        return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("motor_operation");

    node->declare_parameter<std::string>("pidvid",   "0403:6011");
    node->declare_parameter<int>        ("motor_id", 0);

    const std::string pidvid   = node->get_parameter("pidvid").as_string();
    const int         motor_id = node->get_parameter("motor_id").as_int();

    // ── Validate parameters ───────────────────────────────────────────────
    if (motor_id < 0 || motor_id > static_cast<int>(m80106::MAX_MOTOR_ID)) {
        RCLCPP_ERROR(node->get_logger(),
                     "motor_id=%d is out of range [0, %d]. Aborting.",
                     motor_id, static_cast<int>(m80106::MAX_MOTOR_ID));
        rclcpp::shutdown();
        return 1;
    }

    // ── Scan all matching ports ───────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
                "Searching for serial ports matching PID:VID '%s' ...",
                pidvid.c_str());

    const auto scan = m80106_execs::scanAllPorts(pidvid);

    if (scan.ports.empty()) {
        RCLCPP_ERROR(node->get_logger(),
                     "No ports found for PID:VID '%s'. Aborting.", pidvid.c_str());
        rclcpp::shutdown();
        return 1;
    }

    // Per-port results
    for (const auto & ps : scan.ports) {
        RCLCPP_INFO(node->get_logger(),
                    "─────────────────────────────────────────────────────");
        RCLCPP_INFO(node->get_logger(), "Port      : %s", ps.port.c_str());
        RCLCPP_INFO(node->get_logger(), "Hardware  : %s", ps.hardware_id.c_str());
        if (ps.motor_ids.empty()) {
            RCLCPP_WARN(node->get_logger(),
                        "No Unitree actuators responded on %s.", ps.port.c_str());
        } else {
            std::string id_list;
            for (uint8_t id : ps.motor_ids) {
                if (!id_list.empty()) id_list += ", ";
                id_list += std::to_string(static_cast<int>(id));
            }
            RCLCPP_INFO(node->get_logger(), "Actuators : [%s]", id_list.c_str());
        }
    }

    RCLCPP_INFO(node->get_logger(),
                "═════════════════════════════════════════════════════");
    RCLCPP_INFO(node->get_logger(),
                "Total: %zu actuator(s) across %zu port(s).",
                scan.totalMotors(), scan.ports.size());

    // ── Locate target motor ───────────────────────────────────────────────
    const auto all = scan.allMotors();
    auto it = std::find_if(all.begin(), all.end(),
        [&](const m80106_execs::DiscoveredMotor & m) {
            return m.id == static_cast<uint8_t>(motor_id);
        });

    if (it == all.end()) {
        RCLCPP_ERROR(node->get_logger(),
                     "Motor ID %d not found on any port. Aborting.", motor_id);
        rclcpp::shutdown();
        return 1;
    }

    const std::string port  = it->port;
    const std::string hw_id = it->hardware_id;
    const uint8_t id = static_cast<uint8_t>(motor_id);

    RCLCPP_INFO(node->get_logger(),
                "Target motor ID %d confirmed on %s  (hw: %s)",
                motor_id, port.c_str(), hw_id.c_str());
    RCLCPP_INFO(node->get_logger(),
                "Starting motion verification routine (~30 s) ...");
    RCLCPP_INFO(node->get_logger(),
                "═════════════════════════════════════════════════════");

    // ── Open driver ───────────────────────────────────────────────────────
    m80106::MotorDriver driver(port);
    MotorData fb{};

    // ─────────────────────────────────────────────────────────────────────
    // [1/5] BRAKE – read initial state
    // ─────────────────────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
                "[1/5] BRAKE — reading initial state ...");
    brakeFor(driver, id, 500, fb);
    logState(node->get_logger(), fb);
    if (checkError(node->get_logger(), fb)) {
        RCLCPP_ERROR(node->get_logger(), "Hardware error at startup. Aborting.");
        brakeFor(driver, id, 1000, fb);
        rclcpp::shutdown();
        return 1;
    }

    const float start_pos_rotor = fb.correct ? fb.Pos : 0.0f;

    // ─────────────────────────────────────────────────────────────────────
    // [2/5] VELOCITY – ±5 rad/s output, 3 s each direction
    //
    // SDK approach: K_P=0 (no pos gain), K_W=velocity_damping,
    //               W=desired_rotor_speed, T=0, Pos=0
    // ─────────────────────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
                "[2/5] VELOCITY MODE");
    {
        // Kd output-side = 2.0 → rotor-side = 2.0 / r²
        const float K_W = m80106::toRotorKd(2.0f);

        RCLCPP_INFO(node->get_logger(),
                    "  +5 rad/s (output) = %.1f rad/s (rotor) for 3 s ...",
                    m80106::toRotorSpeed(5.0f));
        auto cmd_fwd = makeFOCCmd(id,
            /*T=*/0.0f,
            /*W=*/m80106::toRotorSpeed(5.0f),
            /*Pos=*/0.0f,
            /*K_P=*/0.0f,
            /*K_W=*/K_W);
        runFor(driver, cmd_fwd, 3000, fb);
        logState(node->get_logger(), fb);
        checkError(node->get_logger(), fb);

        // Decelerate to zero before reversing (smooth transition)
        RCLCPP_INFO(node->get_logger(),
                    "  Decelerating to 0 rad/s (500 ms) ...");
        auto cmd_zero = makeFOCCmd(id,
            /*T=*/0.0f, /*W=*/0.0f, /*Pos=*/0.0f,
            /*K_P=*/0.0f, /*K_W=*/K_W);
        runFor(driver, cmd_zero, 500, fb);

        RCLCPP_INFO(node->get_logger(),
                    "  -5 rad/s (output) = %.1f rad/s (rotor) for 3 s ...",
                    m80106::toRotorSpeed(-5.0f));
        auto cmd_rev = makeFOCCmd(id,
            /*T=*/0.0f,
            /*W=*/m80106::toRotorSpeed(-5.0f),
            /*Pos=*/0.0f,
            /*K_P=*/0.0f,
            /*K_W=*/K_W);
        runFor(driver, cmd_rev, 3000, fb);
        logState(node->get_logger(), fb);
        checkError(node->get_logger(), fb);
    }

    // Settle
    RCLCPP_INFO(node->get_logger(), "  Braking to rest (1 s) ...");
    brakeFor(driver, id, 1000, fb);

    // ─────────────────────────────────────────────────────────────────────
    // [3/5] POSITION – PD, ±0.5 rad output, then return to start
    //
    // SDK approach: K_P=stiffness, K_W=damping, Pos=target_rotor_pos, W=0, T=0
    // ─────────────────────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
                "[3/5] POSITION MODE");
    {
        // Refresh current position after the velocity phase
        brakeFor(driver, id, 100, fb);
        const float cur_rotor = fb.correct ? fb.Pos : start_pos_rotor;

        // Kp output-side = 20 → rotor-side = 20 / r²
        const float K_P = m80106::toRotorKp(20.0f);
        // Kd output-side = 1.0 → rotor-side = 1.0 / r²
        const float K_W = m80106::toRotorKd(1.0f);

        const float target_plus  = cur_rotor + m80106::toRotorPos(0.5f);
        const float target_minus = cur_rotor - m80106::toRotorPos(0.5f);

        RCLCPP_INFO(node->get_logger(),
                    "  Moving to +0.5 rad (output), hold 4 s ...");
        auto cmd_plus = makeFOCCmd(id,
            /*T=*/0.0f, /*W=*/0.0f,
            /*Pos=*/target_plus,
            /*K_P=*/K_P, /*K_W=*/K_W);
        runFor(driver, cmd_plus, 4000, fb);
        logState(node->get_logger(), fb);
        checkError(node->get_logger(), fb);

        // Brief settle before the 1.0 rad step to reduce jerk
        RCLCPP_INFO(node->get_logger(),
                    "  Settling at +0.5 rad (300 ms) ...");
        brakeFor(driver, id, 300, fb);

        RCLCPP_INFO(node->get_logger(),
                    "  Moving to -0.5 rad (output), hold 4 s ...");
        auto cmd_minus = makeFOCCmd(id,
            /*T=*/0.0f, /*W=*/0.0f,
            /*Pos=*/target_minus,
            /*K_P=*/K_P, /*K_W=*/K_W);
        runFor(driver, cmd_minus, 4000, fb);
        logState(node->get_logger(), fb);
        checkError(node->get_logger(), fb);

        RCLCPP_INFO(node->get_logger(),
                    "  Returning to start position, hold 3 s ...");
        auto cmd_home = makeFOCCmd(id,
            /*T=*/0.0f, /*W=*/0.0f,
            /*Pos=*/cur_rotor,
            /*K_P=*/K_P, /*K_W=*/K_W);
        runFor(driver, cmd_home, 3000, fb);
        logState(node->get_logger(), fb);
        checkError(node->get_logger(), fb);
    }

    // Settle
    RCLCPP_INFO(node->get_logger(), "  Braking to rest (1 s) ...");
    brakeFor(driver, id, 1000, fb);

    // ─────────────────────────────────────────────────────────────────────
    // [4/5] TORQUE – direct torque with baseline damping
    //
    // SDK approach: K_P=0, K_W≥floor, T=desired_torque, W=0, Pos=0
    // Torque reduced to 0.15 N·m rotor (~0.95 N·m output) to prevent
    // runaway.  Baseline damping floor from makeFOCCmd limits speed.
    // ─────────────────────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
                "[4/5] TORQUE MODE");
    {
        // Record position before torque test for recovery
        brakeFor(driver, id, 100, fb);
        const float pre_torque_rotor = fb.correct ? fb.Pos : start_pos_rotor;

        RCLCPP_INFO(node->get_logger(),
                    "  +0.15 N·m (rotor) for 1.5 s ...");
        auto cmd_pos_trq = makeFOCCmd(id,
            /*T=*/0.15f, /*W=*/0.0f, /*Pos=*/0.0f,
            /*K_P=*/0.0f, /*K_W=*/0.0f);
        runFor(driver, cmd_pos_trq, 1500, fb);
        logState(node->get_logger(), fb);
        checkError(node->get_logger(), fb);

        // Settle before reversing torque direction
        RCLCPP_INFO(node->get_logger(),
                    "  Zero-torque settle (300 ms) ...");
        auto cmd_settle = makeFOCCmd(id,
            /*T=*/0.0f, /*W=*/0.0f, /*Pos=*/0.0f,
            /*K_P=*/0.0f, /*K_W=*/0.0f);
        runFor(driver, cmd_settle, 300, fb);

        RCLCPP_INFO(node->get_logger(),
                    "  -0.15 N·m (rotor) for 1.5 s ...");
        auto cmd_neg_trq = makeFOCCmd(id,
            /*T=*/-0.15f, /*W=*/0.0f, /*Pos=*/0.0f,
            /*K_P=*/0.0f, /*K_W=*/0.0f);
        runFor(driver, cmd_neg_trq, 1500, fb);
        logState(node->get_logger(), fb);
        checkError(node->get_logger(), fb);

        // Return to pre-torque position using PD hold
        RCLCPP_INFO(node->get_logger(),
                    "  Returning to pre-torque position (3 s) ...");
        const float K_P_recovery = m80106::toRotorKp(20.0f);
        const float K_W_recovery = m80106::toRotorKd(1.0f);
        auto cmd_recover = makeFOCCmd(id,
            /*T=*/0.0f, /*W=*/0.0f,
            /*Pos=*/pre_torque_rotor,
            /*K_P=*/K_P_recovery, /*K_W=*/K_W_recovery);
        runFor(driver, cmd_recover, 3000, fb);
        logState(node->get_logger(), fb);
        checkError(node->get_logger(), fb);
    }

    // ─────────────────────────────────────────────────────────────────────
    // [5/5] BRAKE – safe final state
    // ─────────────────────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
                "[5/5] BRAKE — returning motor to safe state (2 s) ...");
    brakeFor(driver, id, 2000, fb);
    logState(node->get_logger(), fb);
    checkError(node->get_logger(), fb);

    RCLCPP_INFO(node->get_logger(),
                "═════════════════════════════════════════════════════");
    RCLCPP_INFO(node->get_logger(),
                "Motion routine complete. Motor safely braked.");
    RCLCPP_INFO(node->get_logger(),
                "═════════════════════════════════════════════════════");

    rclcpp::shutdown();
    return 0;
}
