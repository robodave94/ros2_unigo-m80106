/**
 * @file dual_motor_operation_node.cpp
 * @brief Preprogrammed motion-verification routine for TWO Unitree GO-M8010-6
 *        actuators running simultaneously on a single RS-485 bus.
 *
 * This is the two-motor counterpart of motor_operation_node.cpp.  It scans all
 * RS-485 adapters matching the given PID:VID, locates both target motors
 * (whether they share one bus or sit on separate buses), then executes the
 * identical short (~30 s) motion sequence on both motors at once — every 100 Hz
 * cycle issues the same command to motor A and motor B, so the two motors move
 * together through every control mode:
 *
 *   [1/5] BRAKE    — read initial position and telemetry (both motors).
 *   [2/5] VELOCITY — +5 rad/s output for 3 s, then -5 rad/s for 3 s (both).
 *   [3/5] POSITION — PD: move to +0.5 rad (output), then -0.5 rad,
 *                    then return to start position (both).  [disabled]
 *   [4/5] TORQUE   — direct torque: +1.5 N·m for 2 s, then -1.5 N·m for 2 s.
 *   [5/5] BRAKE    — safe final state (both motors).
 *
 * Total motion time: ~30 s (well under the 60 s target).
 *
 * Because the two motors may share one half-duplex RS-485 bus (or be split
 * across two buses), "simultaneous" means both commands are issued within the
 * same 100 Hz cycle (A then B, sub-ms apart); the routine, direction and timing
 * are identical for both motors.
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
 *   pidvid      (string, default "0403:6011") — USB PID:VID of the RS-485 adapter.
 *   motor_id_a  (int,    default 0)           — First motor ID to operate (0–14).
 *   motor_id_b  (int,    default 1)           — Second motor ID to operate (0–14).
 *
 * Usage:
 *   ros2 run m80106_execs dual_motor_operation --ros-args -p motor_id_a:=0 -p motor_id_b:=1
 *   ros2 run m80106_execs dual_motor_operation --ros-args \
 *       -p motor_id_a:=0 -p motor_id_b:=2 -p pidvid:=0403:6011
 */

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "m80106_lib/motor_driver.hpp"
#include "m80106_lib/motor_types.hpp"

#include "m80106_lib/multi_serial_go8_scanner.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Minimum velocity-damping floor (output-side 0.5 → rotor-side ≈ 0.0125).
/// Applied to every FOC command to prevent uncontrolled acceleration and
/// reduce jerk during setpoint transitions.
static const float MIN_DAMPING_KW = m80106::toRotorKd(0.5f);

/// Build a FOC MotorCmd (all fields rotor-side, clamped to protocol limits).
/// K_W is floored to MIN_DAMPING_KW so no FOC command is ever fully undamped.
static MotorCmd makeFOCCmd(uint8_t id,
                           float T, float W, float Pos,
                           float K_P, float K_W)
{
    MotorCmd cmd;
    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id = id;
    cmd.mode = m80106::toSDKMode(m80106::MotorMode::FOC);
    cmd.T = m80106::clamp(T, -m80106::PROTOCOL_MAX_TORQUE_NM, m80106::PROTOCOL_MAX_TORQUE_NM);
    cmd.W = m80106::clamp(W, -m80106::PROTOCOL_MAX_SPEED_RADS, m80106::PROTOCOL_MAX_SPEED_RADS);
    cmd.Pos = m80106::clamp(Pos, -m80106::PROTOCOL_MAX_POS_RAD, m80106::PROTOCOL_MAX_POS_RAD);
    cmd.K_P = m80106::clamp(K_P, 0.0f, m80106::MAX_KP);
    cmd.K_W = std::max(m80106::clamp(K_W, 0.0f, m80106::MAX_KD), MIN_DAMPING_KW);
    return cmd;
}

/// Send @p cmd_a and @p cmd_b at 100 Hz for @p duration_ms milliseconds.
/// Both commands are issued in every cycle so the two motors move together.
/// Updates @p last_a / @p last_b with the most recent valid feedback received.
/// Send @p cmd_a and @p cmd_b at 100 Hz for @p duration_ms milliseconds.
/// Motor A is driven via @p driver_a, motor B via @p driver_b (these may refer
/// to the same driver when both motors share one bus).  Both commands are
/// issued in every cycle so the two motors move together.
/// Updates @p last_a / @p last_b with the most recent valid feedback received.
static void runFor(m80106::MotorDriver &driver_a,
                   m80106::MotorDriver &driver_b,
                   const MotorCmd &cmd_a,
                   const MotorCmd &cmd_b,
                   int duration_ms,
                   MotorData &last_a,
                   MotorData &last_b)
{
    constexpr int rate_hz = 100;
    constexpr int period_us = 1000000 / rate_hz;
    const int steps = (duration_ms * rate_hz) / 1000;

    for (int i = 0; i < steps && rclcpp::ok(); ++i)
    {
        MotorCmd ca = cmd_a;
        MotorCmd cb = cmd_b;
        MotorData fa;
        MotorData fb;
        if (driver_a.sendRecv(ca, fa) && fa.correct)
        {
            last_a = fa;
        }
        if (driver_b.sendRecv(cb, fb) && fb.correct)
        {
            last_b = fb;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    }
}

/// Issue brake commands to both motors at 100 Hz for @p duration_ms ms.
/// Motor A is driven via @p driver_a, motor B via @p driver_b (which may be the
/// same driver when both motors share one bus).
static void brakeFor(m80106::MotorDriver &driver_a,
                     m80106::MotorDriver &driver_b,
                     uint8_t id_a, uint8_t id_b,
                     int duration_ms,
                     MotorData &last_a, MotorData &last_b)
{
    constexpr int rate_hz = 100;
    constexpr int period_us = 1000000 / rate_hz;
    const int steps = (duration_ms * rate_hz) / 1000;

    for (int i = 0; i < steps && rclcpp::ok(); ++i)
    {
        MotorData fa;
        MotorData fb;
        if (driver_a.brake(id_a, fa) && fa.correct)
        {
            last_a = fa;
        }
        if (driver_b.brake(id_b, fb) && fb.correct)
        {
            last_b = fb;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    }
}

/// Log a one-line telemetry snapshot (output-side values) with a motor label.
static void logState(rclcpp::Logger logger, const char *label, const MotorData &fb)
{
    if (!fb.correct)
    {
        RCLCPP_WARN(logger, "  [%s] (no valid feedback)", label);
        return;
    }
    RCLCPP_INFO(logger,
                "  [%s] ID=%-2d  Mode=%-9s  Pos=%+.3f rad(out)  "
                "Speed=%+.2f rad/s(out)  Torque=%+.2f N·m  Temp=%d°C",
                label,
                static_cast<int>(fb.motor_id),
                m80106::modeString(static_cast<m80106::MotorMode>(fb.mode)),
                m80106::toOutputPos(fb.Pos),
                m80106::toOutputSpeed(fb.W),
                fb.T,
                fb.Temp);
}

/// Check feedback for hardware errors. Returns true if an error was detected.
static bool checkError(rclcpp::Logger logger, const char *label, const MotorData &fb)
{
    if (!fb.correct)
        return false;
    auto err = m80106::toMotorError(fb.MError);
    if (err != m80106::MotorError::NONE)
    {
        RCLCPP_ERROR(logger, "  [%s] *** HARDWARE ERROR: %s (code %d) ***",
                     label, m80106::errorString(err), fb.MError);
        return true;
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dual_motor_operation");

    node->declare_parameter<std::string>("pidvid", "0403:6011");
    node->declare_parameter<int>("motor_id_a", 0);
    node->declare_parameter<int>("motor_id_b", 1);

    const std::string pidvid = node->get_parameter("pidvid").as_string();
    const int motor_id_a = node->get_parameter("motor_id_a").as_int();
    const int motor_id_b = node->get_parameter("motor_id_b").as_int();

    // ── Validate parameters ───────────────────────────────────────────────
    if (motor_id_a < 0 || motor_id_a > static_cast<int>(m80106::MAX_MOTOR_ID))
    {
        RCLCPP_ERROR(node->get_logger(),
                     "motor_id_a=%d is out of range [0, %d]. Aborting.",
                     motor_id_a, static_cast<int>(m80106::MAX_MOTOR_ID));
        rclcpp::shutdown();
        return 1;
    }
    if (motor_id_b < 0 || motor_id_b > static_cast<int>(m80106::MAX_MOTOR_ID))
    {
        RCLCPP_ERROR(node->get_logger(),
                     "motor_id_b=%d is out of range [0, %d]. Aborting.",
                     motor_id_b, static_cast<int>(m80106::MAX_MOTOR_ID));
        rclcpp::shutdown();
        return 1;
    }
    if (motor_id_a == motor_id_b)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "motor_id_a and motor_id_b must differ (both = %d). Aborting.",
                     motor_id_a);
        rclcpp::shutdown();
        return 1;
    }

    // ── Scan all matching ports ───────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
                "Searching for serial ports matching PID:VID '%s' ...",
                pidvid.c_str());

    const auto scan = m80106::scanAllPorts(pidvid);

    if (scan.ports.empty())
    {
        RCLCPP_ERROR(node->get_logger(),
                     "No ports found for PID:VID '%s'. Aborting.", pidvid.c_str());
        rclcpp::shutdown();
        return 1;
    }

    // Per-port results
    for (const auto &ps : scan.ports)
    {
        RCLCPP_INFO(node->get_logger(),
                    "─────────────────────────────────────────────────────");
        RCLCPP_INFO(node->get_logger(), "Port      : %s", ps.port.c_str());
        RCLCPP_INFO(node->get_logger(), "Hardware  : %s", ps.hardware_id.c_str());
        if (ps.motor_ids.empty())
        {
            RCLCPP_WARN(node->get_logger(),
                        "No Unitree actuators responded on %s.", ps.port.c_str());
        }
        else
        {
            std::string id_list;
            for (uint8_t id : ps.motor_ids)
            {
                if (!id_list.empty())
                    id_list += ", ";
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

    // ── Locate both target motors ─────────────────────────────────────────
    const auto all = scan.allMotors();
    auto find_motor = [&](int wanted)
    {
        return std::find_if(all.begin(), all.end(),
                            [&](const m80106::DiscoveredMotor &m)
                            {
                                return m.id == static_cast<uint8_t>(wanted);
                            });
    };

    auto it_a = find_motor(motor_id_a);
    auto it_b = find_motor(motor_id_b);

    if (it_a == all.end())
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Motor ID %d (A) not found on any port. Aborting.", motor_id_a);
        rclcpp::shutdown();
        return 1;
    }
    if (it_b == all.end())
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Motor ID %d (B) not found on any port. Aborting.", motor_id_b);
        rclcpp::shutdown();
        return 1;
    }

    const std::string port_a = it_a->port;
    const std::string port_b = it_b->port;
    const std::string hw_a = it_a->hardware_id;
    const std::string hw_b = it_b->hardware_id;
    const bool same_bus = (port_a == port_b);
    const uint8_t id_a = static_cast<uint8_t>(motor_id_a);
    const uint8_t id_b = static_cast<uint8_t>(motor_id_b);

    RCLCPP_INFO(node->get_logger(),
                "Target motor A: ID %d on %s  (hw: %s)",
                motor_id_a, port_a.c_str(), hw_a.c_str());
    RCLCPP_INFO(node->get_logger(),
                "Target motor B: ID %d on %s  (hw: %s)",
                motor_id_b, port_b.c_str(), hw_b.c_str());
    if (same_bus)
    {
        RCLCPP_INFO(node->get_logger(),
                    "Both motors share bus %s (single driver).", port_a.c_str());
    }
    else
    {
        RCLCPP_INFO(node->get_logger(),
                    "Motors are on separate buses (one driver each).");
    }
    RCLCPP_INFO(node->get_logger(),
                "Starting dual-motor motion verification routine (~30 s) ...");
    RCLCPP_INFO(node->get_logger(),
                "═════════════════════════════════════════════════════");

    // ── Open driver(s) ────────────────────────────────────────────────────
    // One MotorDriver per bus.  When both motors share a bus, a single driver
    // addresses both IDs; otherwise each motor gets its own driver.  driver_a
    // and driver_b are references that alias the same object on a shared bus.
    m80106::MotorDriver driver_a(port_a);
    std::unique_ptr<m80106::MotorDriver> driver_b_owned;
    if (!same_bus)
    {
        driver_b_owned = std::make_unique<m80106::MotorDriver>(port_b);
    }
    m80106::MotorDriver &driver_b = same_bus ? driver_a : *driver_b_owned;

    MotorData fa{};
    MotorData fb{};

    // ─────────────────────────────────────────────────────────────────────
    // [1/5] BRAKE – read initial state
    // ─────────────────────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
                "[1/5] BRAKE — reading initial state ...");
    brakeFor(driver_a, driver_b, id_a, id_b, 500, fa, fb);
    logState(node->get_logger(), "A", fa);
    logState(node->get_logger(), "B", fb);
    if (checkError(node->get_logger(), "A", fa) ||
        checkError(node->get_logger(), "B", fb))
    {
        RCLCPP_ERROR(node->get_logger(), "Hardware error at startup. Aborting.");
        brakeFor(driver_a, driver_b, id_a, id_b, 1000, fa, fb);
        rclcpp::shutdown();
        return 1;
    }

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
        auto cmd_fwd_a = makeFOCCmd(id_a,
                                    /*T=*/0.0f,
                                    /*W=*/m80106::toRotorSpeed(5.0f),
                                    /*Pos=*/0.0f,
                                    /*K_P=*/0.0f,
                                    /*K_W=*/K_W);
        auto cmd_fwd_b = makeFOCCmd(id_b,
                                    /*T=*/0.0f,
                                    /*W=*/m80106::toRotorSpeed(5.0f),
                                    /*Pos=*/0.0f,
                                    /*K_P=*/0.0f,
                                    /*K_W=*/K_W);
        runFor(driver_a, driver_b, cmd_fwd_a, cmd_fwd_b, 3000, fa, fb);
        logState(node->get_logger(), "A", fa);
        logState(node->get_logger(), "B", fb);
        checkError(node->get_logger(), "A", fa);
        checkError(node->get_logger(), "B", fb);

        // Decelerate to zero before reversing (smooth transition)
        RCLCPP_INFO(node->get_logger(),
                    "  Decelerating to 0 rad/s (500 ms) ...");
        auto cmd_zero_a = makeFOCCmd(id_a,
                                     /*T=*/0.0f, /*W=*/0.0f, /*Pos=*/0.0f,
                                     /*K_P=*/0.0f, /*K_W=*/K_W);
        auto cmd_zero_b = makeFOCCmd(id_b,
                                     /*T=*/0.0f, /*W=*/0.0f, /*Pos=*/0.0f,
                                     /*K_P=*/0.0f, /*K_W=*/K_W);
        runFor(driver_a, driver_b, cmd_zero_a, cmd_zero_b, 500, fa, fb);

        RCLCPP_INFO(node->get_logger(),
                    "  -5 rad/s (output) = %.1f rad/s (rotor) for 3 s ...",
                    m80106::toRotorSpeed(-5.0f));
        auto cmd_rev_a = makeFOCCmd(id_a,
                                    /*T=*/0.0f,
                                    /*W=*/m80106::toRotorSpeed(-5.0f),
                                    /*Pos=*/0.0f,
                                    /*K_P=*/0.0f,
                                    /*K_W=*/K_W);
        auto cmd_rev_b = makeFOCCmd(id_b,
                                    /*T=*/0.0f,
                                    /*W=*/m80106::toRotorSpeed(-5.0f),
                                    /*Pos=*/0.0f,
                                    /*K_P=*/0.0f,
                                    /*K_W=*/K_W);
        runFor(driver_a, driver_b, cmd_rev_a, cmd_rev_b, 3000, fa, fb);
        logState(node->get_logger(), "A", fa);
        logState(node->get_logger(), "B", fb);
        checkError(node->get_logger(), "A", fa);
        checkError(node->get_logger(), "B", fb);
    }

    // Settle
    RCLCPP_INFO(node->get_logger(), "  Braking to rest (1 s) ...");
    brakeFor(driver_a, driver_b, id_a, id_b, 1000, fa, fb);

    // ─────────────────────────────────────────────────────────────────────
    // [3/5] POSITION – PD, ±0.5 rad output, then return to start
    //
    // SDK approach: K_P=stiffness, K_W=damping, Pos=target_rotor_pos, W=0, T=0
    // ─────────────────────────────────────────────────────────────────────
    // RCLCPP_INFO(node->get_logger(),
    //             "[3/5] POSITION MODE");
    // {
    //     // Refresh current position after the velocity phase
    //     brakeFor(driver_a, driver_b, id_a, id_b, 100, fa, fb);
    //     const float cur_rotor_a = fa.correct ? fa.Pos : 0.0f;
    //     const float cur_rotor_b = fb.correct ? fb.Pos : 0.0f;

    //     // Kp output-side = 20 → rotor-side = 20 / r²
    //     const float K_P = m80106::toRotorKp(20.0f);
    //     // Kd output-side = 1.0 → rotor-side = 1.0 / r²
    //     const float K_W = m80106::toRotorKd(1.0f);

    //     const float target_plus_a  = cur_rotor_a + m80106::toRotorPos(0.5f);
    //     const float target_plus_b  = cur_rotor_b + m80106::toRotorPos(0.5f);
    //     const float target_minus_a = cur_rotor_a - m80106::toRotorPos(0.5f);
    //     const float target_minus_b = cur_rotor_b - m80106::toRotorPos(0.5f);

    //     RCLCPP_INFO(node->get_logger(),
    //                 "  Moving to +0.5 rad (output), hold 4 s ...");
    //     auto cmd_plus_a = makeFOCCmd(id_a,
    //         /*T=*/0.0f, /*W=*/0.0f, /*Pos=*/target_plus_a,
    //         /*K_P=*/K_P, /*K_W=*/K_W);
    //     auto cmd_plus_b = makeFOCCmd(id_b,
    //         /*T=*/0.0f, /*W=*/0.0f, /*Pos=*/target_plus_b,
    //         /*K_P=*/K_P, /*K_W=*/K_W);
    //     runFor(driver_a, driver_b, cmd_plus_a, cmd_plus_b, 4000, fa, fb);
    //     logState(node->get_logger(), "A", fa);
    //     logState(node->get_logger(), "B", fb);
    //     checkError(node->get_logger(), "A", fa);
    //     checkError(node->get_logger(), "B", fb);

    //     // Brief settle before the next step to reduce jerk
    //     RCLCPP_INFO(node->get_logger(),
    //                 "  Settling at +0.5 rad (300 ms) ...");
    //     brakeFor(driver_a, driver_b, id_a, id_b, 300, fa, fb);

    //     RCLCPP_INFO(node->get_logger(),
    //                 "  Moving to -0.5 rad (output), hold 4 s ...");
    //     auto cmd_minus_a = makeFOCCmd(id_a,
    //         /*T=*/0.0f, /*W=*/0.0f, /*Pos=*/target_minus_a,
    //         /*K_P=*/K_P, /*K_W=*/K_W);
    //     auto cmd_minus_b = makeFOCCmd(id_b,
    //         /*T=*/0.0f, /*W=*/0.0f, /*Pos=*/target_minus_b,
    //         /*K_P=*/K_P, /*K_W=*/K_W);
    //     runFor(driver_a, driver_b, cmd_minus_a, cmd_minus_b, 4000, fa, fb);
    //     logState(node->get_logger(), "A", fa);
    //     logState(node->get_logger(), "B", fb);
    //     checkError(node->get_logger(), "A", fa);
    //     checkError(node->get_logger(), "B", fb);

    //     RCLCPP_INFO(node->get_logger(),
    //                 "  Returning to start position, hold 3 s ...");
    //     auto cmd_home_a = makeFOCCmd(id_a,
    //         /*T=*/0.0f, /*W=*/0.0f, /*Pos=*/cur_rotor_a,
    //         /*K_P=*/K_P, /*K_W=*/K_W);
    //     auto cmd_home_b = makeFOCCmd(id_b,
    //         /*T=*/0.0f, /*W=*/0.0f, /*Pos=*/cur_rotor_b,
    //         /*K_P=*/K_P, /*K_W=*/K_W);
    //     runFor(driver_a, driver_b, cmd_home_a, cmd_home_b, 3000, fa, fb);
    //     logState(node->get_logger(), "A", fa);
    //     logState(node->get_logger(), "B", fb);
    //     checkError(node->get_logger(), "A", fa);
    //     checkError(node->get_logger(), "B", fb);
    // }

    // // Settle
    // RCLCPP_INFO(node->get_logger(), "  Braking to rest (1 s) ...");
    // brakeFor(driver_a, driver_b, id_a, id_b, 1000, fa, fb);

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
        brakeFor(driver_a, driver_b, id_a, id_b, 100, fa, fb);
        const float pre_torque_rotor_a = fa.correct ? fa.Pos : 0.0f;
        const float pre_torque_rotor_b = fb.correct ? fb.Pos : 0.0f;

        RCLCPP_INFO(node->get_logger(),
                    "  +0.15 N·m (rotor) for 1.5 s ...");
        auto cmd_pos_trq_a = makeFOCCmd(id_a,
                                        /*T=*/0.15f, /*W=*/0.0f, /*Pos=*/0.0f,
                                        /*K_P=*/0.0f, /*K_W=*/0.0f);
        auto cmd_pos_trq_b = makeFOCCmd(id_b,
                                        /*T=*/0.15f, /*W=*/0.0f, /*Pos=*/0.0f,
                                        /*K_P=*/0.0f, /*K_W=*/0.0f);
        runFor(driver_a, driver_b, cmd_pos_trq_a, cmd_pos_trq_b, 1500, fa, fb);
        logState(node->get_logger(), "A", fa);
        logState(node->get_logger(), "B", fb);
        checkError(node->get_logger(), "A", fa);
        checkError(node->get_logger(), "B", fb);

        // Settle before reversing torque direction
        RCLCPP_INFO(node->get_logger(),
                    "  Zero-torque settle (300 ms) ...");
        auto cmd_settle_a = makeFOCCmd(id_a,
                                       /*T=*/0.0f, /*W=*/0.0f, /*Pos=*/0.0f,
                                       /*K_P=*/0.0f, /*K_W=*/0.0f);
        auto cmd_settle_b = makeFOCCmd(id_b,
                                       /*T=*/0.0f, /*W=*/0.0f, /*Pos=*/0.0f,
                                       /*K_P=*/0.0f, /*K_W=*/0.0f);
        runFor(driver_a, driver_b, cmd_settle_a, cmd_settle_b, 300, fa, fb);

        RCLCPP_INFO(node->get_logger(),
                    "  -0.15 N·m (rotor) for 1.5 s ...");
        auto cmd_neg_trq_a = makeFOCCmd(id_a,
                                        /*T=*/-0.15f, /*W=*/0.0f, /*Pos=*/0.0f,
                                        /*K_P=*/0.0f, /*K_W=*/0.0f);
        auto cmd_neg_trq_b = makeFOCCmd(id_b,
                                        /*T=*/-0.15f, /*W=*/0.0f, /*Pos=*/0.0f,
                                        /*K_P=*/0.0f, /*K_W=*/0.0f);
        runFor(driver_a, driver_b, cmd_neg_trq_a, cmd_neg_trq_b, 1500, fa, fb);
        logState(node->get_logger(), "A", fa);
        logState(node->get_logger(), "B", fb);
        checkError(node->get_logger(), "A", fa);
        checkError(node->get_logger(), "B", fb);

        // Return to pre-torque position using PD hold
        RCLCPP_INFO(node->get_logger(),
                    "  Returning to pre-torque position (3 s) ...");
        const float K_P_recovery = m80106::toRotorKp(20.0f);
        const float K_W_recovery = m80106::toRotorKd(1.0f);
        auto cmd_recover_a = makeFOCCmd(id_a,
                                        /*T=*/0.0f, /*W=*/0.0f,
                                        /*Pos=*/pre_torque_rotor_a,
                                        /*K_P=*/K_P_recovery, /*K_W=*/K_W_recovery);
        auto cmd_recover_b = makeFOCCmd(id_b,
                                        /*T=*/0.0f, /*W=*/0.0f,
                                        /*Pos=*/pre_torque_rotor_b,
                                        /*K_P=*/K_P_recovery, /*K_W=*/K_W_recovery);
        runFor(driver_a, driver_b, cmd_recover_a, cmd_recover_b, 3000, fa, fb);
        logState(node->get_logger(), "A", fa);
        logState(node->get_logger(), "B", fb);
        checkError(node->get_logger(), "A", fa);
        checkError(node->get_logger(), "B", fb);
    }

    // ─────────────────────────────────────────────────────────────────────
    // [5/5] BRAKE – safe final state
    // ─────────────────────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
                "[5/5] BRAKE — returning motors to safe state (2 s) ...");
    brakeFor(driver_a, driver_b, id_a, id_b, 2000, fa, fb);
    logState(node->get_logger(), "A", fa);
    logState(node->get_logger(), "B", fb);
    checkError(node->get_logger(), "A", fa);
    checkError(node->get_logger(), "B", fb);

    RCLCPP_INFO(node->get_logger(),
                "═════════════════════════════════════════════════════");
    RCLCPP_INFO(node->get_logger(),
                "Dual-motor routine complete. Both motors safely braked.");
    RCLCPP_INFO(node->get_logger(),
                "═════════════════════════════════════════════════════");

    rclcpp::shutdown();
    return 0;
}
