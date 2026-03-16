/**
 * @file tui.cpp
 * @brief FTXUI-based interactive terminal UI for GO-M8010-6 motor management.
 *
 * Replaces the former ncurses implementation with FTXUI v5, fetched at build
 * time via CMake FetchContent.  No system TUI library is required.
 *
 * Layout:
 *   ┌─ Header ───────────────────────────────────────────────────────────────┐
 *   ├─ RS-485 Bus Panel (scrollable motor table)  ─────────────────────────  ┤
 *   ├─ Control Panel (PD gains, mode, setpoints, apply) ──────────────────── ┤
 *   ├─ Status bar ──────────────────────────────────────────────────────────┤
 *   └────────────────────────────────────────────────────────────────────────┘
 */

#include "m80106_bringup/tui.hpp"

#include <ftxui/component/component.hpp>
#include <ftxui/component/component_base.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/color.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "m80106_lib/motor_types.hpp"

using namespace ftxui;

namespace m80106_bringup {

// ─────────────────────────────────────────────────────────────────────────────
// Constants / helpers
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int MODE_COUNT = 3;
static const char * const MODE_NAMES[MODE_COUNT] = {"BRAKE", "FOC", "CALIBRATE"};

static std::string fmtFloat(float v, const char * fmt = "%+.4f")
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), fmt, v);
    return buf;
}

static Color tempColor(int t)
{
    if (t >= m80106::TEMP_PROTECTION_CELSIUS) { return Color::Red; }
    if (t >= m80106::TEMP_WARNING_CELSIUS)    { return Color::Yellow; }
    return Color::GreenLight;
}

static Color errorColor(m80106::MotorError e)
{
    return (e == m80106::MotorError::NONE) ? Color::GreenLight : Color::Red;
}

// ─────────────────────────────────────────────────────────────────────────────
// Action helpers
// ─────────────────────────────────────────────────────────────────────────────

static void doRefresh(m80106::MotorDriver & driver, AppState & state)
{
    state.status_msg = "Scanning...";

    const std::vector<uint8_t> found = driver.scanMotors();
    state.discovered_ids = found;

    std::vector<MotorState> new_states;
    new_states.reserve(found.size());

    for (uint8_t id : found) {
        MotorState ms;
        ms.id = id;

        for (const auto & old : state.motor_states) {
            if (old.id == id) { ms = old; break; }
        }

        MotorData fb;
        if (driver.brake(id, fb) && fb.correct) {
            ms.id         = fb.motor_id;
            ms.mode       = static_cast<m80106::MotorMode>(fb.mode);
            ms.error      = m80106::toMotorError(fb.MError);
            ms.pos_rad    = fb.Pos;
            ms.speed_rads = fb.W;
            ms.torque_nm  = fb.T;
            ms.temp_c     = fb.Temp;
            ms.foot_force = fb.footForce;
            ms.valid      = true;
        }
        new_states.push_back(ms);
    }

    state.motor_states = std::move(new_states);

    if (!state.motor_states.empty()) {
        state.selected_motor_idx = std::min(
            state.selected_motor_idx,
            static_cast<int>(state.motor_states.size()) - 1);
    } else {
        state.selected_motor_idx = 0;
    }

    char msg[64];
    std::snprintf(msg, sizeof(msg), "Found %d motor(s).",
                  static_cast<int>(found.size()));
    state.status_msg = msg;
}

static void populateCtrlFromMotor(const MotorState & ms, AppState & state)
{
    state.selected_mode_idx = static_cast<int>(ms.mode);
    state.buf_kp  = "0.0";
    state.buf_kd  = "0.0";
    state.buf_pos = fmtFloat(ms.pos_rad);
    state.buf_w   = fmtFloat(ms.speed_rads);
    state.buf_t   = fmtFloat(ms.torque_nm);
}

static void applyControlCmd(m80106::MotorDriver & driver, AppState & state)
{
    const MotorState * ms = state.selectedMotor();
    if (!ms) {
        state.status_msg = "No motor selected.";
        return;
    }

    const float kp  = std::strtof(state.buf_kp.c_str(),  nullptr);
    const float kd  = std::strtof(state.buf_kd.c_str(),  nullptr);
    const float pos = std::strtof(state.buf_pos.c_str(), nullptr);
    const float w   = std::strtof(state.buf_w.c_str(),   nullptr);
    const float t   = std::strtof(state.buf_t.c_str(),   nullptr);

    const float c_kp  = m80106::clamp(kp,  0.0f, m80106::MAX_KP);
    const float c_kd  = m80106::clamp(kd,  0.0f, m80106::MAX_KD);
    const float c_pos = m80106::clamp(pos, -m80106::MAX_POS_RAD,    m80106::MAX_POS_RAD);
    const float c_w   = m80106::clamp(w,   -m80106::MAX_SPEED_RADS, m80106::MAX_SPEED_RADS);
    const float c_t   = m80106::clamp(t,   -m80106::MAX_TORQUE_NM,  m80106::MAX_TORQUE_NM);

    const auto mode = static_cast<m80106::MotorMode>(
        std::min(state.selected_mode_idx, 2));

    MotorCmd cmd;
    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id        = ms->id;
    cmd.mode      = m80106::toSDKMode(mode);
    cmd.T         = c_t;
    cmd.W         = c_w;
    cmd.Pos       = c_pos;
    cmd.K_P       = c_kp;
    cmd.K_W       = c_kd;

    MotorData fb;
    const bool ok = driver.sendRecv(cmd, fb);

    if (ok && fb.correct) {
        MotorState & entry =
            state.motor_states[static_cast<std::size_t>(state.selected_motor_idx)];
        entry.mode       = static_cast<m80106::MotorMode>(fb.mode);
        entry.error      = m80106::toMotorError(fb.MError);
        entry.pos_rad    = fb.Pos;
        entry.speed_rads = fb.W;
        entry.torque_nm  = fb.T;
        entry.temp_c     = fb.Temp;
        entry.foot_force = fb.footForce;
        entry.valid      = true;

        char msg[80];
        std::snprintf(msg, sizeof(msg), "OK  Motor %d  Pos=%+.3f W=%+.3f T=%+.3f",
                      ms->id, fb.Pos, fb.W, fb.T);
        state.status_msg = msg;
    } else {
        char msg[48];
        std::snprintf(msg, sizeof(msg), "Send failed for motor %d", ms->id);
        state.status_msg = msg;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Render helpers — pure functions returning FTXUI Elements
// ─────────────────────────────────────────────────────────────────────────────

static Element renderHeader(const m80106::BusInfo & info)
{
    auto conn_elem = info.connected
        ? text(" CONNECTED ") | bold | color(Color::Green)
        : text(" DISCONNECTED ") | bold | color(Color::Red);

    return hbox({
        text(" M80106 Motor UI") | bold | color(Color::Cyan),
        text("  |  Port: "),
        text(info.port_path.empty() ? "N/A" : info.port_path) | color(Color::White),
        text("  |  HW: "),
        text(info.hardware_id.empty() ? "N/A" : info.hardware_id) | color(Color::White),
        text("  |  "),
        conn_elem,
        filler(),
    }) | bgcolor(Color::Black);
}

static Element renderBusRow(const MotorState & ms, bool selected, bool panel_focused)
{
    // Highlight colour when selected
    const Decorator highlight = selected && panel_focused
        ? (Decorator)(color(Color::Black) | bgcolor(Color::Cyan))
        : selected
            ? (Decorator)(color(Color::White) | bgcolor(Color::GrayDark))
            : nothing;

    auto cell = [&](std::string s, int w) -> Element {
        while (static_cast<int>(s.size()) < w) { s += ' '; }
        if (static_cast<int>(s.size()) > w)    { s  = s.substr(0, w); }
        return text(s) | highlight;
    };

    char id_buf[8];
    std::snprintf(id_buf, sizeof(id_buf), " %2d ", ms.id);

    const Color temp_c  = tempColor(ms.temp_c);
    const Color error_c = errorColor(ms.error);
    char tmp_buf[12]; std::snprintf(tmp_buf, sizeof(tmp_buf), "%3d\xc2\xb0""C", ms.temp_c);

    return hbox({
        text(selected ? ">" : " ") | (selected ? bold : nothing),
        cell(id_buf,                             5),
        cell(m80106::modeString(ms.mode),       11),
        cell(fmtFloat(ms.pos_rad),              13),
        cell(fmtFloat(ms.speed_rads),           13),
        cell(fmtFloat(ms.torque_nm),            13),
        selected ? cell(tmp_buf, 8)
                 : text(tmp_buf) | color(temp_c),
        text("  "),
        selected ? cell(m80106::errorString(ms.error), 15)
                 : text(m80106::errorString(ms.error)) | color(error_c),
    });
}

static Element renderBusPanel(const AppState & state, bool focused)
{
    Elements rows;

    rows.push_back(hbox({
        text("  ID   Mode        Pos(rad)      Spd(r/s)      Trq(Nm)       Temp      Error")
            | bold | color(Color::Cyan),
    }));
    rows.push_back(separator());

    if (state.motor_states.empty()) {
        rows.push_back(
            text("  No motors found. Press 'r' to scan the bus.") | color(Color::Yellow)
        );
    } else {
        for (int i = 0; i < static_cast<int>(state.motor_states.size()); ++i) {
            rows.push_back(renderBusRow(
                state.motor_states[static_cast<std::size_t>(i)],
                i == state.selected_motor_idx,
                focused));
        }
    }

    const std::string title = focused ? " > RS-485 Bus " : " RS-485 Bus ";
    return window(text(title) | bold, vbox(rows));
}

static Element renderCtrlField(const std::string & label,
                                const std::string & value,
                                bool active)
{
    auto box = hbox({
        text("["),
        text(value.empty() ? " " : value),
        text("]"),
    });
    if (active) { box = box | color(Color::Black) | bgcolor(Color::Yellow); }
    return hbox({ text(label), box });
}

static Element renderControlPanel(const AppState & state, bool focused)
{
    const MotorState * ms = state.selectedMotor();
    Elements content;

    if (!ms) {
        content.push_back(
            text("  Select a motor in the Bus Panel (Tab + up/down, then Enter).")
                | color(Color::Yellow)
        );
    } else {
        const int  cf = state.active_ctrl_field;
        const bool fp = focused;

        content.push_back(hbox({
            text("  "),
            renderCtrlField("Mode: ", MODE_NAMES[state.selected_mode_idx],
                fp && cf == static_cast<int>(CtrlField::MODE)),
            text("  (m=cycle)  "),
            renderCtrlField("Kp: ", state.buf_kp,
                fp && cf == static_cast<int>(CtrlField::KP)),
            text("  "),
            renderCtrlField("Kd: ", state.buf_kd,
                fp && cf == static_cast<int>(CtrlField::KD)),
        }));

        content.push_back(hbox({
            text("  "),
            renderCtrlField("Pos(rad): ", state.buf_pos,
                fp && cf == static_cast<int>(CtrlField::TARGET_POS)),
            text("  "),
            renderCtrlField("W(r/s): ", state.buf_w,
                fp && cf == static_cast<int>(CtrlField::TARGET_W)),
            text("  "),
            renderCtrlField("T(Nm): ", state.buf_t,
                fp && cf == static_cast<int>(CtrlField::TARGET_T)),
        }));

        content.push_back(
            text("  Kp/Kd 0-25.599  Pos +/-411774rad  W +/-804r/s  T +/-127.99Nm")
                | color(Color::GrayLight)
        );

        const bool apply_focused = fp && (cf == CTRL_FIELD_COUNT);
        auto apply_btn = text(" Apply (Enter) ");
        if (apply_focused) {
            apply_btn = apply_btn | bold | color(Color::Black) | bgcolor(Color::Green);
        } else {
            apply_btn = apply_btn | bold | color(Color::Green);
        }

        Element live_elem = filler();
        if (ms->valid) {
            char live[128];
            std::snprintf(live, sizeof(live),
                "  Live: Pos=%+.3f W=%+.3f T=%+.3f  Temp=%d°C  %s",
                ms->pos_rad, ms->speed_rads, ms->torque_nm,
                ms->temp_c, m80106::errorString(ms->error));
            live_elem = text(live) | color(Color::Cyan);
        }

        content.push_back(hbox({
            text("  ["), apply_btn, text("]  "), live_elem,
        }));
    }

    char title_buf[64];
    if (ms) {
        std::snprintf(title_buf, sizeof(title_buf),
            "%s Control — Motor %d", focused ? " > " : "   ", ms->id);
    } else {
        std::snprintf(title_buf, sizeof(title_buf),
            "%s Control — (no motor selected)", focused ? " > " : "   ");
    }

    return window(text(title_buf) | bold, vbox(content));
}

static Element renderStatusBar(const std::string & msg)
{
    return hbox({
        text(" r=Refresh  Tab=Panel  Up/Dn=Nav  Enter=Sel/Apply"
             "  m=Mode  0-9=Edit  BS=Del  ?=Help  q=Quit")
            | bold | color(Color::White) | bgcolor(Color::Blue),
        filler() | bgcolor(Color::Blue),
        text("  " + msg + "  ")
            | bold | color(Color::Yellow) | bgcolor(Color::Blue),
    });
}

static Element renderHelp()
{
    return vbox({
        text("+-Help-----------------------------------------------------------+") | bold,
        text("|  r / F5     Refresh: scan bus, poll all motors                |"),
        text("|  Tab        Switch focus: Bus Panel <-> Control Panel          |"),
        text("|  Up / Down  Move selection (motor row / control field)         |"),
        text("|  Enter      Bus: select motor  |  Control: apply command       |"),
        text("|  0-9  .  -  Edit active numeric field in Control Panel         |"),
        text("|  Backspace  Delete last character in active field               |"),
        text("|  m          Cycle mode: BRAKE -> FOC -> CALIBRATE               |"),
        text("|  ?          Toggle this help overlay                            |"),
        text("|  q / Esc    Quit (motors left in current state)                 |"),
        text("|                                                                 |"),
        text("|  Notes:                                                         |"),
        text("|   All values are rotor-side unless noted.                       |"),
        text("|   Output Kp/Kd: divide by 6.33^2 = 40.07                      |"),
        text("|   CALIBRATE: do not send further commands for >= 5 seconds.     |"),
        text("+----------------------------------------------------------------+") | bold,
    }) | color(Color::White) | bgcolor(Color::Blue);
}

// ─────────────────────────────────────────────────────────────────────────────
// TuiApp::run
// ─────────────────────────────────────────────────────────────────────────────

void TuiApp::run(m80106::MotorDriver & driver,
                 const m80106::BusInfo & bus_info,
                 AppState & state)
{
    auto screen = ScreenInteractive::Fullscreen();

    auto root = CatchEvent(
        Renderer([&]() -> Element {
            const bool bus_focused  = (state.active_panel == 0);
            const bool ctrl_focused = (state.active_panel == 1);

            Element layout = vbox({
                renderHeader(bus_info),
                separator(),
                renderBusPanel(state, bus_focused) | flex,
                renderControlPanel(state, ctrl_focused),
                renderStatusBar(state.status_msg),
            });

            if (state.show_help) {
                layout = dbox({
                    layout,
                    vbox({
                        filler(),
                        hbox({ filler(), renderHelp(), filler() }),
                        filler(),
                    }),
                });
            }

            return layout;
        }),
        [&](Event ev) -> bool {
            if (ev == Event::Character('q') || ev == Event::Character('Q') ||
                ev == Event::Escape)
            {
                screen.ExitLoopClosure()();
                return true;
            }

            if (ev == Event::Character('?')) {
                state.show_help = !state.show_help;
                return true;
            }

            if (ev == Event::Character('r') || ev == Event::F5) {
                doRefresh(driver, state);
                return true;
            }

            if (ev == Event::Tab || ev == Event::TabReverse) {
                state.active_panel = (state.active_panel == 0) ? 1 : 0;
                return true;
            }

            if (ev == Event::ArrowUp) {
                if (state.active_panel == 0) {
                    if (state.selected_motor_idx > 0) { --state.selected_motor_idx; }
                } else {
                    if (state.active_ctrl_field > 0) { --state.active_ctrl_field; }
                }
                return true;
            }

            if (ev == Event::ArrowDown) {
                if (state.active_panel == 0) {
                    if (!state.motor_states.empty() &&
                        state.selected_motor_idx <
                            static_cast<int>(state.motor_states.size()) - 1)
                    {
                        ++state.selected_motor_idx;
                    }
                } else {
                    if (state.active_ctrl_field < CTRL_FIELD_COUNT) {
                        ++state.active_ctrl_field;
                    }
                }
                return true;
            }

            if (ev == Event::Return) {
                if (state.active_panel == 0) {
                    const MotorState * ms = state.selectedMotor();
                    if (ms) {
                        populateCtrlFromMotor(*ms, state);
                        state.active_panel      = 1;
                        state.active_ctrl_field = 0;
                    }
                } else {
                    applyControlCmd(driver, state);
                }
                return true;
            }

            if (ev == Event::Character('m') || ev == Event::Character('M')) {
                if (state.active_panel == 1) {
                    state.selected_mode_idx =
                        (state.selected_mode_idx + 1) % MODE_COUNT;
                }
                return true;
            }

            if (ev == Event::Backspace) {
                if (state.active_panel == 1) {
                    std::string * buf = state.activeBuffer();
                    if (buf && !buf->empty()) { buf->pop_back(); }
                }
                return true;
            }

            // Printable numeric chars (0-9, '.', '-', '+') edit the active field
            if (state.active_panel == 1) {
                std::string * buf = state.activeBuffer();
                if (buf) {
                    const std::string & raw = ev.input();
                    if (raw.size() == 1) {
                        const char c = raw[0];
                        const bool numeric = (c >= '0' && c <= '9') ||
                                             c == '.' || c == '-' || c == '+';
                        if (numeric &&
                            static_cast<int>(buf->size()) < FIELD_MAX_LEN)
                        {
                            buf->push_back(c);
                            return true;
                        }
                    }
                }
            }

            return false;
        }
    );

    // Periodic 200 ms screen refresh so live telemetry updates
    std::atomic<bool> running{true};
    std::thread refresh_thread([&]() {
        while (running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            screen.PostEvent(Event::Custom);
        }
    });

    screen.Loop(root);

    running.store(false);
    refresh_thread.join();
}

} // namespace m80106_bringup
