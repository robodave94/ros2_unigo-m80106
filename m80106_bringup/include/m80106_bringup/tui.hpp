#pragma once

#include "m80106_lib/motor_bus.hpp"
#include "m80106_lib/motor_driver.hpp"
#include "m80106_bringup/app_state.hpp"

namespace m80106_bringup {

/**
 * @brief htop-style interactive terminal UI for GO-M8010-6 motor control.
 *
 * Built on FTXUI (https://github.com/ArthurSonzogni/FTXUI v5), a modern
 * C++ TUI library that requires no system installation — it is fetched at
 * build time via CMake FetchContent.
 *
 * Layout (adapts to terminal size):
 *
 *   ┌─ Header ──────────────────────────────────────────────────────────────┐
 *   │  Port: /dev/ttyUSB0  HW: USB VID:PID=1a86:7523  CONNECTED            │
 *   ├─ RS-485 Bus Panel ────────────────────────────────────────────────────┤
 *   │  ID  Mode       Pos(rad)    Spd(r/s)    Trq(Nm)   Temp   Error       │
 *   │   0  BRAKE      +0.000      +0.000      +0.000     25°C  OK          │
 *   │ > 1  FOC        +1.234      +0.001      +0.050     31°C  OK          │
 *   ├─ Control Panel ───────────────────────────────────────────────────────┤
 *   │  Mode:[FOC       ] (m=cycle)  Kp:[0.0000      ]  Kd:[0.0000      ]   │
 *   │  Pos(rad):[0.0000     ]  W(r/s):[0.0000     ]  T(Nm):[0.0000     ]   │
 *   │  Kp/Kd 0–25.599  Pos ±411774rad  W ±804r/s  T ±127.99Nm             │
 *   │  [ Apply (Enter) ]   Live: Pos=+1.234 W=+0.001 T=+0.050 Temp=31°C   │
 *   ├─ Status bar ──────────────────────────────────────────────────────────┤
 *   │  r=Refresh  Tab=Panel  ↑↓=Navigate  Enter=Select/Apply  ?=Help  q=Quit│
 *   └───────────────────────────────────────────────────────────────────────┘
 *
 * Keyboard bindings:
 *   r / F5       — Refresh: scan bus and poll all discovered motors
 *   Tab          — Toggle focus: Bus Panel ↔ Control Panel
 *   ↑ / ↓        — Move selection (motor row / control field)
 *   Enter        — Bus panel: select motor; Control panel: apply command
 *   0-9  .  -    — Edit active numeric field in Control Panel
 *   Backspace    — Delete last character in active field
 *   m            — Cycle control mode (BRAKE → FOC → CALIBRATE)
 *   ?            — Toggle help overlay
 *   q / Esc      — Quit
 */
class TuiApp
{
public:
    TuiApp()  = default;
    ~TuiApp() = default;

    // Non-copyable/non-movable (owns ScreenInteractive internally)
    TuiApp(const TuiApp &)             = delete;
    TuiApp & operator=(const TuiApp &) = delete;

    /**
     * @brief Run the interactive TUI. Blocks until the user quits.
     *
     * FTXUI's ScreenInteractive takes over the terminal for the duration of
     * this call and restores it automatically on return — no explicit
     * init/shutdown pair is needed.
     *
     * @param driver   Motor driver for the open RS-485 port.
     * @param bus_info Adapter metadata shown in the header.
     * @param state    Application state (modified in place by user actions).
     */
    void run(m80106::MotorDriver & driver,
             const m80106::BusInfo & bus_info,
             AppState & state);
};

} // namespace m80106_bringup

