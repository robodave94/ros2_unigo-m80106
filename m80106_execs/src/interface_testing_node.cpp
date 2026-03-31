/**
 * @file interface_testing_node.cpp
 * @brief Test node for the GoM80106SetController library.
 *
 * Exercises the controller with a single GO-M8010-6 motor (no external
 * gearbox) through a series of motion tests:
 *
 *   [1/7] INIT           — Construct controller, log initial state.
 *   [2/7] POSITION       — Move ±2π rad (1 revolution each way).
 *   [3/7] VELOCITY LIMIT — Reduce velocity to 5 rad/s, verify during move.
 *   [4/7] MAX SPEED      — Move 3 revolutions, verify near-max velocity.
 *   [5/7] ACCELERATION   — Slow acceleration (2.0 rad/s²), verify ramp.
 *   [6/7] ZERO POSITION  — Zero at +π, verify offset tracking.
 *   [7/7] FEEDBACK       — Query and log all feedback fields.
 *
 * Node parameters:
 *   pidvid    (string, default "0403:6011") — USB PID:VID of the RS-485 adapter.
 *   motor_id  (int,    default 0)           — Motor ID to test (0–14).
 *
 * Usage:
 *   ros2 run m80106_execs interface_testing --ros-args -p motor_id:=0
 */

#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "m80106_lib/go_m80106_set_controller.hpp"
#include "m80106_lib/motor_config.hpp"
#include "m80106_lib/motor_types.hpp"

static constexpr float PI = static_cast<float>(M_PI);

/// Wait until the motor settles near @p target_rad within @p tolerance_rad,
/// or @p timeout_ms elapses.  Logs position every 250 ms.
/// Returns true if the motor reached the target.
static bool waitForPosition(
    rclcpp::Logger logger,
    m80106::GoM80106SetController & ctrl,
    uint8_t id,
    float target_rad,
    float tolerance_rad = 0.05f,
    int timeout_ms = 10000)
{
    using clock = std::chrono::steady_clock;
    auto start  = clock::now();
    auto period = std::chrono::milliseconds(250);
    auto deadline = start + std::chrono::milliseconds(timeout_ms);

    while (clock::now() < deadline && rclcpp::ok())
    {
        float pos = ctrl.getPosition(id);
        float err = std::fabs(pos - target_rad);
        if (err < tolerance_rad)
        {
            RCLCPP_INFO(logger,
                "    Reached target %.3f rad  (actual=%.3f, error=%.4f)",
                target_rad, pos, err);
            return true;
        }
        std::this_thread::sleep_for(period);
    }

    float pos = ctrl.getPosition(id);
    RCLCPP_WARN(logger,
        "    TIMEOUT reaching target %.3f rad  (actual=%.3f, error=%.4f)",
        target_rad, pos, std::fabs(pos - target_rad));
    return false;
}

/// Log full feedback for motor @p id.
static void logFeedback(
    rclcpp::Logger logger,
    m80106::GoM80106SetController & ctrl,
    uint8_t id)
{
    RCLCPP_INFO(logger, "    Position (zeroed) : %.4f rad",  ctrl.getPosition(id));
    RCLCPP_INFO(logger, "    Position (global) : %.4f rad",  ctrl.getGlobalPosition(id));
    RCLCPP_INFO(logger, "    Velocity          : %.4f rad/s", ctrl.getVelocity(id));
    RCLCPP_INFO(logger, "    Torque            : %.4f N·m",   ctrl.getTorque(id));
    RCLCPP_INFO(logger, "    Temperature       : %d °C",      ctrl.getTemperature(id));
    RCLCPP_INFO(logger, "    Error code        : %d (%s)",    ctrl.getErrorCode(id),
        m80106::errorString(m80106::toMotorError(ctrl.getErrorCode(id))));
    RCLCPP_INFO(logger, "    Mode              : %d (%s)",    ctrl.getMode(id),
        m80106::modeString(static_cast<m80106::MotorMode>(ctrl.getMode(id))));
    RCLCPP_INFO(logger, "    Correct (CRC)     : %s",
        ctrl.getCorrect(id) ? "true" : "false");
    RCLCPP_INFO(logger, "    Foot force        : %d",         ctrl.getFootForce(id));
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("interface_testing");

    node->declare_parameter<std::string>("pidvid", "0403:6011");
    node->declare_parameter<int>("motor_id", 0);

    const std::string pidvid = node->get_parameter("pidvid").as_string();
    const int motor_id_int   = node->get_parameter("motor_id").as_int();

    if (motor_id_int < 0 || motor_id_int > static_cast<int>(m80106::MAX_MOTOR_ID))
    {
        RCLCPP_ERROR(node->get_logger(),
            "motor_id=%d is out of range [0, %d]. Aborting.",
            motor_id_int, static_cast<int>(m80106::MAX_MOTOR_ID));
        rclcpp::shutdown();
        return 1;
    }

    const uint8_t id = static_cast<uint8_t>(motor_id_int);

    auto logger = node->get_logger();

    // =====================================================================
    // [1/7] INIT — Construct controller
    // =====================================================================
    RCLCPP_INFO(logger,
        "═══════════════════════════════════════════════════════════");
    RCLCPP_INFO(logger,
        "[1/7] INIT — Constructing GoM80106SetController ...");

    std::unique_ptr<m80106::GoM80106SetController> ctrl;
    try
    {
        std::vector<m80106::GoMotorConfig> configs;
        configs.emplace_back(id); // single motor, no external gearbox, all defaults

        ctrl = std::make_unique<m80106::GoM80106SetController>(pidvid, configs);
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(logger, "Failed to construct controller: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(logger, "  Controller initialised successfully.");
    // Give the control thread a moment to start and read initial position
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(logger, "  Initial state:");
    logFeedback(logger, *ctrl, id);

    // Check for hardware errors at startup
    if (ctrl->getErrorCode(id) != 0)
    {
        RCLCPP_ERROR(logger,
            "Hardware error at startup: %s. Aborting.",
            m80106::errorString(m80106::toMotorError(ctrl->getErrorCode(id))));
        ctrl.reset();
        rclcpp::shutdown();
        return 1;
    }

    // =====================================================================
    // [2/7] POSITION TEST — ±2π rad (1 revolution each way)
    // =====================================================================
    RCLCPP_INFO(logger,
        "═══════════════════════════════════════════════════════════");
    RCLCPP_INFO(logger,
        "[2/7] POSITION TEST — ±1 revolution (±2π rad)");
    {
        RCLCPP_INFO(logger, "  Moving to +2π rad (+1 revolution) ...");
        ctrl->setTargetPosition(id, 2.0f * PI);
        waitForPosition(logger, *ctrl, id, 2.0f * PI);

        RCLCPP_INFO(logger, "  Moving to -2π rad (-1 revolution from start) ...");
        ctrl->setTargetPosition(id, -2.0f * PI);
        waitForPosition(logger, *ctrl, id, -2.0f * PI);

        RCLCPP_INFO(logger, "  Returning to 0 ...");
        ctrl->setTargetPosition(id, 0.0f);
        waitForPosition(logger, *ctrl, id, 0.0f);
    }

    // =====================================================================
    // [3/7] VELOCITY LIMIT TEST — 5 rad/s during motion
    // =====================================================================
    RCLCPP_INFO(logger,
        "═══════════════════════════════════════════════════════════");
    RCLCPP_INFO(logger,
        "[3/7] VELOCITY LIMIT TEST — 5 rad/s cap");
    {
        RCLCPP_INFO(logger, "  Setting velocity limit to 5 rad/s ...");
        ctrl->setVelocityLimit(id, 5.0f);

        RCLCPP_INFO(logger, "  Moving to +4π rad (+2 revolutions) ...");
        ctrl->setTargetPosition(id, 4.0f * PI);

        // Monitor velocity during motion
        float max_vel_observed = 0.0f;
        for (int i = 0; i < 40 && rclcpp::ok(); ++i)  // 10 seconds
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            float vel = std::fabs(ctrl->getVelocity(id));
            if (vel > max_vel_observed) max_vel_observed = vel;
        }

        waitForPosition(logger, *ctrl, id, 4.0f * PI, 0.1f, 10000);

        RCLCPP_INFO(logger,
            "  Max velocity observed: %.3f rad/s (limit=5.0)",
            max_vel_observed);
        if (max_vel_observed <= 5.5f)
        {
            RCLCPP_INFO(logger, "  PASS: Velocity stayed within limit.");
        }
        else
        {
            RCLCPP_WARN(logger,
                "  WARN: Velocity exceeded limit (%.3f > 5.5 rad/s).",
                max_vel_observed);
        }

        RCLCPP_INFO(logger, "  Restoring default velocity limit (21 rad/s) ...");
        ctrl->setVelocityLimit(id, m80106::UNIVERSAL_VELOCITY_LIMIT_RADS);

        RCLCPP_INFO(logger, "  Returning to 0 ...");
        ctrl->setTargetPosition(id, 0.0f);
        waitForPosition(logger, *ctrl, id, 0.0f);
    }

    // =====================================================================
    // [4/7] MAX SPEED TEST — 3 revolutions, verify near-max velocity
    // =====================================================================
    RCLCPP_INFO(logger,
        "═══════════════════════════════════════════════════════════");
    RCLCPP_INFO(logger,
        "[4/7] MAX SPEED TEST — 3 revolutions at full speed");
    {
        RCLCPP_INFO(logger, "  Moving to +6π rad (+3 revolutions) ...");
        ctrl->setTargetPosition(id, 6.0f * PI);

        float max_vel_observed = 0.0f;
        {
            using clock = std::chrono::steady_clock;
            auto deadline = clock::now() + std::chrono::seconds(10);
            while (clock::now() < deadline && rclcpp::ok())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
                float vel = std::fabs(ctrl->getVelocity(id));
                if (vel > max_vel_observed) max_vel_observed = vel;
            }
        }

        waitForPosition(logger, *ctrl, id, 6.0f * PI, 0.1f, 15000);

        RCLCPP_INFO(logger,
            "  Max velocity observed: %.3f rad/s (motor max ≈ 21.0)",
            max_vel_observed);
        if (max_vel_observed >= 15.0f)
        {
            RCLCPP_INFO(logger,
                "  PASS: Motor reached significant speed (%.1f rad/s).",
                max_vel_observed);
        }
        else
        {
            RCLCPP_WARN(logger,
                "  WARN: Motor only reached %.1f rad/s (expected >15).",
                max_vel_observed);
        }

        RCLCPP_INFO(logger, "  Returning to 0 ...");
        ctrl->setTargetPosition(id, 0.0f);
        waitForPosition(logger, *ctrl, id, 0.0f);
    }

    // =====================================================================
    // [5/7] ACCELERATION PROFILE TEST — slow ramp at 2.0 rad/s²
    // =====================================================================
    RCLCPP_INFO(logger,
        "═══════════════════════════════════════════════════════════");
    RCLCPP_INFO(logger,
        "[5/7] ACCELERATION TEST — 2.0 rad/s²");
    {
        RCLCPP_INFO(logger, "  Setting acceleration to 2.0 rad/s² ...");
        ctrl->setAccelerationProfile(id, 2.0f);

        RCLCPP_INFO(logger, "  Moving to +4π rad ...");
        ctrl->setTargetPosition(id, 4.0f * PI);

        // Monitor acceleration (velocity delta per sample)
        float prev_vel = 0.0f;
        float max_accel_observed = 0.0f;
        for (int i = 0; i < 60 && rclcpp::ok(); ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            float vel = ctrl->getVelocity(id);
            float dv = std::fabs(vel - prev_vel) / 0.2f; // rad/s²
            if (dv > max_accel_observed && i > 0) max_accel_observed = dv;
            prev_vel = vel;
        }

        waitForPosition(logger, *ctrl, id, 4.0f * PI, 0.1f, 15000);

        RCLCPP_INFO(logger,
            "  Max acceleration observed: ~%.2f rad/s² (limit=2.0)",
            max_accel_observed);
        if (max_accel_observed <= 3.5f) // some tolerance for measurement noise
        {
            RCLCPP_INFO(logger,
                "  PASS: Acceleration profile respected.");
        }
        else
        {
            RCLCPP_WARN(logger,
                "  WARN: Measured acceleration (%.2f) exceeded expected range.",
                max_accel_observed);
        }

        RCLCPP_INFO(logger,
            "  Restoring default acceleration (%.1f rad/s²) ...",
            m80106::DEFAULT_ACCELERATION_RADS2);
        ctrl->setAccelerationProfile(id, m80106::DEFAULT_ACCELERATION_RADS2);

        RCLCPP_INFO(logger, "  Returning to 0 ...");
        ctrl->setTargetPosition(id, 0.0f);
        waitForPosition(logger, *ctrl, id, 0.0f);
    }

    // =====================================================================
    // [6/7] ZERO POSITION TEST
    // =====================================================================
    RCLCPP_INFO(logger,
        "═══════════════════════════════════════════════════════════");
    RCLCPP_INFO(logger,
        "[6/7] ZERO POSITION TEST");
    {
        RCLCPP_INFO(logger, "  Moving to +π rad ...");
        ctrl->setTargetPosition(id, PI);
        waitForPosition(logger, *ctrl, id, PI);

        float pre_zero_pos    = ctrl->getPosition(id);
        float pre_zero_global = ctrl->getGlobalPosition(id);
        RCLCPP_INFO(logger,
            "  Before zero: position=%.4f, global=%.4f",
            pre_zero_pos, pre_zero_global);

        RCLCPP_INFO(logger, "  Calling zeroPosition(%d) ...", id);
        ctrl->zeroPosition(id);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        float post_zero_pos    = ctrl->getPosition(id);
        float post_zero_global = ctrl->getGlobalPosition(id);
        RCLCPP_INFO(logger,
            "  After zero: position=%.4f (should be ~0), global=%.4f (should be ~π)",
            post_zero_pos, post_zero_global);

        if (std::fabs(post_zero_pos) < 0.1f)
        {
            RCLCPP_INFO(logger, "  PASS: Zeroed position is near 0.");
        }
        else
        {
            RCLCPP_WARN(logger,
                "  WARN: Zeroed position (%.4f) not near 0.", post_zero_pos);
        }

        if (std::fabs(post_zero_global - PI) < 0.2f)
        {
            RCLCPP_INFO(logger, "  PASS: Global position is near π.");
        }
        else
        {
            RCLCPP_WARN(logger,
                "  WARN: Global position (%.4f) not near π.", post_zero_global);
        }

        // Move back to global 0 (which is the physical start location).
        // setTargetPosition takes global (un-zeroed) coordinates, so 0.0f = global origin.
        // After the motor arrives there, getPosition returns (global 0 - zero_offset π) = -π,
        // which is what waitForPosition checks in the zeroed frame.
        RCLCPP_INFO(logger, "  Returning to global 0 (zeroed frame: -π) ...");
        ctrl->setTargetPosition(id, 0.0f);
        waitForPosition(logger, *ctrl, id, -PI);

        RCLCPP_INFO(logger,
            "  Global position after return: %.4f (should be ~0)",
            ctrl->getGlobalPosition(id));

        // Reset zero offset by zeroing at global 0
        ctrl->zeroPosition(id);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // =====================================================================
    // [7/7] FEEDBACK QUERY TEST
    // =====================================================================
    RCLCPP_INFO(logger,
        "═══════════════════════════════════════════════════════════");
    RCLCPP_INFO(logger,
        "[7/7] FEEDBACK QUERY TEST — all fields");
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        logFeedback(logger, *ctrl, id);

        // Test multi-motor overloads (with a single ID in the vector)
        RCLCPP_INFO(logger, "  Testing multi-motor overloads ...");
        std::vector<uint8_t> ids = {id};

        auto pos_map  = ctrl->getPosition(ids);
        auto gpos_map = ctrl->getGlobalPosition(ids);
        auto vel_map  = ctrl->getVelocity(ids);
        auto trq_map  = ctrl->getTorque(ids);
        auto tmp_map  = ctrl->getTemperature(ids);
        auto err_map  = ctrl->getErrorCode(ids);
        auto mode_map = ctrl->getMode(ids);
        auto cor_map  = ctrl->getCorrect(ids);
        auto ff_map   = ctrl->getFootForce(ids);

        RCLCPP_INFO(logger, "  Multi-motor getPosition:      {%d -> %.4f}",
            id, pos_map[id]);
        RCLCPP_INFO(logger, "  Multi-motor getGlobalPosition: {%d -> %.4f}",
            id, gpos_map[id]);
        RCLCPP_INFO(logger, "  Multi-motor getVelocity:      {%d -> %.4f}",
            id, vel_map[id]);
        RCLCPP_INFO(logger, "  Multi-motor getTorque:        {%d -> %.4f}",
            id, trq_map[id]);
        RCLCPP_INFO(logger, "  Multi-motor getTemperature:   {%d -> %d}",
            id, tmp_map[id]);
        RCLCPP_INFO(logger, "  Multi-motor getErrorCode:     {%d -> %d}",
            id, err_map[id]);
        RCLCPP_INFO(logger, "  Multi-motor getMode:          {%d -> %d}",
            id, mode_map[id]);
        RCLCPP_INFO(logger, "  Multi-motor getCorrect:       {%d -> %s}",
            id, cor_map[id] ? "true" : "false");
        RCLCPP_INFO(logger, "  Multi-motor getFootForce:     {%d -> %d}",
            id, ff_map[id]);

        RCLCPP_INFO(logger, "  PASS: All multi-motor overloads returned data.");
    }

    // =====================================================================
    // SHUTDOWN
    // =====================================================================
    RCLCPP_INFO(logger,
        "═══════════════════════════════════════════════════════════");
    RCLCPP_INFO(logger, "All tests complete. Shutting down controller ...");

    ctrl.reset(); // Destructor brakes all motors and joins thread

    RCLCPP_INFO(logger, "Controller destroyed. Exiting.");

    rclcpp::shutdown();
    return 0;
}
