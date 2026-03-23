/**
 * @file scan_motor_information_node.cpp
 * @brief Real-time htop-style terminal monitor for Unitree GO-M8010-6 motors.
 *
 * Uses the shared multi_serial_go8_scanner to discover all USB-serial ports
 * and motors, then continuously polls each motor's telemetry via brake
 * commands and renders data in a live, colour-coded terminal UI.
 *
 * Node parameters:
 *   pidvid      (string, default "0403:6011") — USB PID:VID to match.
 *   target_id   (int,    default -1)          — If >=0, monitor only this ID.
 *   rate_hz     (double, default 20.0)        — Poll / refresh rate (Hz).
 *
 * Press 'q' to quit.
 *
 * Usage:
 *   ros2 run m80106_execs scan_motor_info
 *   ros2 run m80106_execs scan_motor_info --ros-args -p target_id:=0
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "m80106_lib/motor_driver.hpp"
#include "m80106_lib/motor_types.hpp"

#include "m80106_execs/multi_serial_go8_scanner.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// ANSI escape helpers
// ─────────────────────────────────────────────────────────────────────────────

namespace ansi {
    constexpr const char * RESET      = "\033[0m";
    constexpr const char * BOLD       = "\033[1m";
    constexpr const char * DIM        = "\033[2m";
    constexpr const char * UNDERLINE  = "\033[4m";
    constexpr const char * FG_BLACK   = "\033[30m";
    constexpr const char * FG_RED     = "\033[31m";
    constexpr const char * FG_GREEN   = "\033[32m";
    constexpr const char * FG_YELLOW  = "\033[33m";
    constexpr const char * FG_BLUE    = "\033[34m";
    constexpr const char * FG_MAGENTA = "\033[35m";
    constexpr const char * FG_CYAN    = "\033[36m";
    constexpr const char * FG_WHITE   = "\033[37m";
    constexpr const char * BG_BLACK   = "\033[40m";
    constexpr const char * BG_BLUE    = "\033[44m";
    constexpr const char * BG_GREEN   = "\033[42m";
    constexpr const char * CLEAR      = "\033[2J";
    constexpr const char * HOME       = "\033[H";
    constexpr const char * HIDE_CUR   = "\033[?25l";
    constexpr const char * SHOW_CUR   = "\033[?25h";
}

// ─────────────────────────────────────────────────────────────────────────────
// Terminal raw-mode for non-blocking key detection
// ─────────────────────────────────────────────────────────────────────────────

namespace {

struct RawTerminal {
    struct termios orig_;
    bool active_ = false;

    void enable() {
        if (active_) return;
        tcgetattr(STDIN_FILENO, &orig_);
        struct termios raw = orig_;
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        active_ = true;
    }

    void disable() {
        if (!active_) return;
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
        active_ = false;
    }

    ~RawTerminal() { disable(); }

    /// Return next key or -1 if nothing available.
    int readKey() const {
        char c = 0;
        if (read(STDIN_FILENO, &c, 1) == 1) return c;
        return -1;
    }
};

/// Get terminal width (columns).
int termWidth() {
    struct winsize w{};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == 0 && w.ws_col > 0) {
        return w.ws_col;
    }
    return 80;
}

std::atomic<bool> g_quit{false};

void signalHandler(int) { g_quit.store(true); }

// ─────────────────────────────────────────────────────────────────────────────
// Motor telemetry snapshot
// ─────────────────────────────────────────────────────────────────────────────

struct MotorSnapshot {
    uint8_t     id           = 0;
    std::string port;
    std::string hardware_id;
    bool        online       = false;  // responded this cycle
    uint8_t     mode         = 0;
    float       torque_nm    = 0.0f;   // rotor
    float       speed_rads   = 0.0f;   // rotor
    float       pos_rad      = 0.0f;   // rotor
    int         temp_c       = 0;
    int         error        = 0;
    int         foot_force   = 0;
    uint64_t    poll_count   = 0;
    uint64_t    error_count  = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Rendering
// ─────────────────────────────────────────────────────────────────────────────

/// Pad or truncate @p s to exactly @p w characters.
std::string pad(const std::string & s, int w) {
    if (static_cast<int>(s.size()) >= w) return s.substr(0, static_cast<size_t>(w));
    return s + std::string(static_cast<size_t>(w) - s.size(), ' ');
}

/// Right-align @p s to @p w characters.
std::string rpad(const std::string & s, int w) {
    if (static_cast<int>(s.size()) >= w) return s.substr(0, static_cast<size_t>(w));
    return std::string(static_cast<size_t>(w) - s.size(), ' ') + s;
}

std::string fmtFloat(float v, int decimals = 2) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, static_cast<double>(v));
    return buf;
}

const char * modeColor(uint8_t mode) {
    switch (mode) {
        case 0: return ansi::FG_YELLOW;  // BRAKE
        case 1: return ansi::FG_GREEN;   // FOC
        case 2: return ansi::FG_MAGENTA; // CALIBRATE
        default: return ansi::FG_RED;
    }
}

const char * tempColor(int temp_c) {
    if (temp_c >= m80106::TEMP_PROTECTION_CELSIUS) return ansi::FG_RED;
    if (temp_c >= m80106::TEMP_WARNING_CELSIUS)    return ansi::FG_YELLOW;
    return ansi::FG_GREEN;
}

const char * errorColor(int err) {
    return (err == 0) ? ansi::FG_GREEN : ansi::FG_RED;
}

void renderHeader(int width, size_t motor_count, double hz, uint64_t cycle) {
    // Title bar (htop-style cyan on black)
    std::printf("%s%s%s", ansi::BOLD, ansi::BG_BLUE, ansi::FG_WHITE);
    std::printf("%s", pad("  GO-M8010-6 Motor Monitor", width).c_str());
    std::printf("%s\n", ansi::RESET);

    // Status bar
    std::printf("%s%s", ansi::BOLD, ansi::FG_CYAN);
    std::printf("  Motors: %zu", motor_count);
    std::printf("    Rate: %.0f Hz", hz);
    std::printf("    Cycle: %lu", static_cast<unsigned long>(cycle));

    // Right-align quit hint
    char hint[] = "  [q] Quit  ";
    int used = 10 + 12 + 14 + 12;  // approximate width of the status items
    int spaces = width - used - static_cast<int>(sizeof(hint));
    if (spaces > 0) {
        std::printf("%*s", spaces, "");
    }
    std::printf("%s%s%s", ansi::FG_YELLOW, hint, ansi::RESET);
    std::printf("\n");
}

void renderTableHeader(int /*width*/) {
    std::printf("%s%s", ansi::BOLD, ansi::UNDERLINE);
    std::printf(" %3s │ %-9s │ %5s │ %12s │ %12s │ %10s │ %10s │ %5s │ %-12s │ %5s │ %-16s",
                "ID", "Mode", "Temp", "Position(r)", "Position(o)",
                "Speed(r)", "Speed(o)", "Trq", "Error", "Force", "Port");
    std::printf("%s\n", ansi::RESET);
}

void renderMotorRow(const MotorSnapshot & m) {
    if (!m.online) {
        // Offline motor — dim
        std::printf("%s", ansi::DIM);
        std::printf(" %3d │ %-9s │ %5s │ %12s │ %12s │ %10s │ %10s │ %5s │ %-12s │ %5s │ %-16s",
                    m.id, "OFFLINE", "--", "--", "--", "--", "--", "--", "--", "--",
                    m.port.c_str());
        std::printf("%s\n", ansi::RESET);
        return;
    }

    // Mode
    const char * mode_str = m80106::modeString(static_cast<m80106::MotorMode>(m.mode));

    // Output-side values
    float out_pos   = m80106::toOutputPos(m.pos_rad);
    float out_speed = m80106::toOutputSpeed(m.speed_rads);

    // Error
    const char * err_str = m80106::errorString(m80106::toMotorError(m.error));

    // ID
    std::printf(" %s%3d%s", ansi::FG_CYAN, m.id, ansi::RESET);

    // Mode (colored)
    std::printf(" │ %s%-9s%s", modeColor(m.mode), mode_str, ansi::RESET);

    // Temp (colored)
    std::printf(" │ %s%4d°C%s", tempColor(m.temp_c), m.temp_c, ansi::RESET);

    // Position rotor
    std::printf(" │ %s%s",  ansi::FG_WHITE, rpad(fmtFloat(m.pos_rad, 3), 12).c_str());
    std::printf("%s", ansi::RESET);

    // Position output
    std::printf(" │ %s%s",  ansi::FG_WHITE, rpad(fmtFloat(out_pos, 3), 12).c_str());
    std::printf("%s", ansi::RESET);

    // Speed rotor
    std::printf(" │ %s%s",  ansi::FG_WHITE, rpad(fmtFloat(m.speed_rads, 2), 10).c_str());
    std::printf("%s", ansi::RESET);

    // Speed output
    std::printf(" │ %s%s",  ansi::FG_WHITE, rpad(fmtFloat(out_speed, 2), 10).c_str());
    std::printf("%s", ansi::RESET);

    // Torque
    std::printf(" │ %s%s",  ansi::FG_WHITE, rpad(fmtFloat(m.torque_nm, 2), 5).c_str());
    std::printf("%s", ansi::RESET);

    // Error (colored)
    std::printf(" │ %s%-12s%s", errorColor(m.error), err_str, ansi::RESET);

    // Foot force
    std::printf(" │ %s%s",  ansi::DIM, rpad(std::to_string(m.foot_force), 5).c_str());
    std::printf("%s", ansi::RESET);

    // Port (short)
    std::string short_port = m.port;
    if (short_port.rfind("/dev/", 0) == 0) {
        short_port = short_port.substr(5);
    }
    std::printf(" │ %s%-16s%s", ansi::DIM, short_port.c_str(), ansi::RESET);

    std::printf("\n");
}

void renderFooter(int width) {
    // Print a horizontal rule using ASCII dash
    for (int i = 0; i < width; ++i) std::putchar('-');
    std::putchar('\n');
    std::printf("%s  Units: Position=rad  Speed=rad/s  Torque=N·m  Temp=°C  (r)=rotor (o)=output%s\n",
                ansi::DIM, ansi::RESET);
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("scan_motor_info");

    node->declare_parameter<std::string>("pidvid", "0403:6011");
    node->declare_parameter<int>("target_id", -1);
    node->declare_parameter<double>("rate_hz", 20.0);

    const std::string pidvid    = node->get_parameter("pidvid").as_string();
    const int         target_id = node->get_parameter("target_id").as_int();
    const double      rate_hz   = node->get_parameter("rate_hz").as_double();

    // ── Initial scan ──────────────────────────────────────────────────────
    RCLCPP_INFO(node->get_logger(),
                "Scanning for motors on PID:VID '%s' ...", pidvid.c_str());

    const auto scan = m80106_execs::scanAllPorts(pidvid);
    auto all_motors = scan.allMotors();

    if (all_motors.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No motors found. Exiting.");
        rclcpp::shutdown();
        return 1;
    }

    // Filter to target if specified
    if (target_id >= 0) {
        auto it = std::remove_if(all_motors.begin(), all_motors.end(),
            [&](const m80106_execs::DiscoveredMotor & m) {
                return m.id != static_cast<uint8_t>(target_id);
            });
        all_motors.erase(it, all_motors.end());
        if (all_motors.empty()) {
            RCLCPP_ERROR(node->get_logger(),
                         "target_id=%d not found on any port. Exiting.", target_id);
            rclcpp::shutdown();
            return 1;
        }
    }

    RCLCPP_INFO(node->get_logger(),
                "Monitoring %zu motor(s). Press 'q' to quit.", all_motors.size());

    // ── Open MotorDriver per port ─────────────────────────────────────────
    // Multiple motors may share the same port/driver.
    struct PortDriver {
        std::string port;
        std::unique_ptr<m80106::MotorDriver> driver;
    };
    std::vector<PortDriver> drivers;
    {
        // Deduplicate ports
        std::vector<std::string> unique_ports;
        for (const auto & m : all_motors) {
            if (std::find(unique_ports.begin(), unique_ports.end(), m.port) == unique_ports.end()) {
                unique_ports.push_back(m.port);
            }
        }
        for (const auto & port : unique_ports) {
            try {
                drivers.push_back({port,
                    std::make_unique<m80106::MotorDriver>(port)});
            } catch (const std::exception & e) {
                RCLCPP_ERROR(node->get_logger(),
                             "Failed to open %s: %s", port.c_str(), e.what());
            }
        }
    }

    if (drivers.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open any serial port. Exiting.");
        rclcpp::shutdown();
        return 1;
    }

    // Build snapshot list
    std::vector<MotorSnapshot> snapshots;
    for (const auto & m : all_motors) {
        MotorSnapshot snap;
        snap.id          = m.id;
        snap.port        = m.port;
        snap.hardware_id = m.hardware_id;
        snapshots.push_back(snap);
    }

    // Helper: find driver for port
    auto findDriver = [&](const std::string & port) -> m80106::MotorDriver* {
        for (auto & pd : drivers) {
            if (pd.port == port) return pd.driver.get();
        }
        return nullptr;
    };

    // ── Setup terminal ────────────────────────────────────────────────────
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    RawTerminal term;
    term.enable();

    // Hide cursor, clear screen
    std::printf("%s%s", ansi::HIDE_CUR, ansi::CLEAR);

    const auto period = std::chrono::microseconds(
        static_cast<int64_t>(1000000.0 / rate_hz));

    uint64_t cycle = 0;

    // ── Main loop ─────────────────────────────────────────────────────────
    while (!g_quit.load() && rclcpp::ok()) {
        auto t0 = std::chrono::steady_clock::now();

        // Check for 'q'
        int key = term.readKey();
        if (key == 'q' || key == 'Q') break;

        // Poll each motor
        for (auto & snap : snapshots) {
            m80106::MotorDriver * drv = findDriver(snap.port);
            if (!drv) {
                snap.online = false;
                continue;
            }

            MotorData data;
            bool ok = drv->brake(snap.id, data);
            snap.poll_count++;

            if (ok && data.correct) {
                snap.online     = true;
                snap.mode       = data.mode;
                snap.torque_nm  = data.T;
                snap.speed_rads = data.W;
                snap.pos_rad    = data.Pos;
                snap.temp_c     = data.Temp;
                snap.error      = data.MError;
                snap.foot_force = data.footForce;
            } else {
                snap.online = false;
                snap.error_count++;
            }
        }

        // ── Render ────────────────────────────────────────────────────────
        int width = termWidth();

        // Move to top-left (no full clear to reduce flicker)
        std::printf("%s", ansi::HOME);

        renderHeader(width, snapshots.size(), rate_hz, cycle);
        std::printf("\n");
        renderTableHeader(width);

        for (const auto & snap : snapshots) {
            renderMotorRow(snap);
        }

        std::printf("\n");
        renderFooter(width);

        std::fflush(stdout);

        ++cycle;

        // Sleep remainder of period
        auto elapsed = std::chrono::steady_clock::now() - t0;
        auto remaining = period - std::chrono::duration_cast<std::chrono::microseconds>(elapsed);
        if (remaining.count() > 0) {
            std::this_thread::sleep_for(remaining);
        }
    }

    // ── Cleanup ───────────────────────────────────────────────────────────
    term.disable();
    std::printf("%s%s", ansi::SHOW_CUR, ansi::RESET);
    std::printf("\n\nMonitor stopped. %zu motor(s) were tracked.\n", snapshots.size());

    rclcpp::shutdown();
    return 0;
}
