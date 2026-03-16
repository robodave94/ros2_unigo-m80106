#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <cstring>

#include "m80106_lib/motor_types.hpp"

namespace m80106_bringup {

/// Telemetry snapshot for one motor — mirrors MotorData but uses typed enums.
struct MotorState
{
    uint8_t           id         = 0;
    m80106::MotorMode mode       = m80106::MotorMode::BRAKE;
    m80106::MotorError error     = m80106::MotorError::NONE;
    float             pos_rad    = 0.0f;   ///< Rotor-side position  [rad]
    float             speed_rads = 0.0f;   ///< Rotor-side speed     [rad/s]
    float             torque_nm  = 0.0f;   ///< Rotor-side torque    [N·m]
    int               temp_c     = 0;      ///< Temperature          [°C]
    int               foot_force = 0;      ///< Foot ADC             [0-4095]
    bool              valid      = false;  ///< true after first successful poll
};

/// Maximum length of any numeric input field (characters typed by the user).
static constexpr int FIELD_MAX_LEN = 15;

/**
 * @brief Indices into the control panel field array.
 *
 * Used as `active_ctrl_field` to know which buffer/value the cursor is in.
 */
enum class CtrlField : int {
    MODE = 0,
    KP,
    KD,
    TARGET_POS,
    TARGET_W,
    TARGET_T,
    COUNT   // sentinel — number of editable fields
};

static constexpr int CTRL_FIELD_COUNT = static_cast<int>(CtrlField::COUNT);

/**
 * @brief Shared mutable state for the TUI application.
 *
 * All fields are owned by the main thread (the TUI event loop). The driver
 * is accessed only from the main thread (on-demand refresh / apply), so no
 * locking is needed.
 */
struct AppState
{
    // ── Bus view ──────────────────────────────────────────────────────────
    std::vector<uint8_t>    discovered_ids;   ///< IDs found by scanMotors()
    std::vector<MotorState> motor_states;     ///< Parallel to discovered_ids

    // ── Selection ─────────────────────────────────────────────────────────
    int  selected_motor_idx = 0;   ///< Highlighted row in bus panel (0-based)

    // ── Panel focus ───────────────────────────────────────────────────────
    /// 0 = bus panel has focus, 1 = control panel has focus.
    int  active_panel = 0;

    // ── Control panel input ───────────────────────────────────────────────
    int  active_ctrl_field = 0;    ///< Which field is being edited (CtrlField)
    int  selected_mode_idx = 0;    ///< Index into mode_names[] (0=BRAKE,1=FOC,2=CAL)

    std::string buf_kp  = "0.0";    ///< Kp gain input
    std::string buf_kd  = "0.0";    ///< Kd gain input
    std::string buf_pos = "0.0";    ///< Rotor-side target position [rad]
    std::string buf_w   = "0.0";    ///< Rotor-side target speed    [rad/s]
    std::string buf_t   = "0.0";    ///< Rotor-side target torque   [N·m]

    // ── UI feedback ───────────────────────────────────────────────────────
    std::string status_msg = "Press 'r' to scan bus  |  '?' for help";
    bool        show_help  = false;

    // ── Helpers ───────────────────────────────────────────────────────────

    /// Return pointer to the active string buffer, or nullptr for MODE or Apply fields.
    std::string * activeBuffer()
    {
        switch (static_cast<CtrlField>(active_ctrl_field)) {
            case CtrlField::KP:         return &buf_kp;
            case CtrlField::KD:         return &buf_kd;
            case CtrlField::TARGET_POS: return &buf_pos;
            case CtrlField::TARGET_W:   return &buf_w;
            case CtrlField::TARGET_T:   return &buf_t;
            default:                    return nullptr;
        }
    }

    /// Non-null if a motor is selected and state entry exists.
    MotorState * selectedMotor()
    {
        if (selected_motor_idx >= 0 &&
            selected_motor_idx < static_cast<int>(motor_states.size()))
        {
            return &motor_states[static_cast<std::size_t>(selected_motor_idx)];
        }
        return nullptr;
    }

    const MotorState * selectedMotor() const
    {
        if (selected_motor_idx >= 0 &&
            selected_motor_idx < static_cast<int>(motor_states.size()))
        {
            return &motor_states[static_cast<std::size_t>(selected_motor_idx)];
        }
        return nullptr;
    }
};

} // namespace m80106_bringup
