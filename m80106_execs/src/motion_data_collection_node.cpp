/**
 * @file motion_data_collection_node.cpp
 * @brief Interactive data-collection CLI for a single Unitree GO-M8010-6 motor.
 *
 * Connects to a target motor (retrying the scan a few times, since the
 * RS-485 link occasionally needs a couple of attempts), then interactively
 * asks the operator how many rotations to perform, a trajectory name, a
 * direction and a speed. The motor is then driven with a velocity command
 * while position/speed/torque/temperature/error/foot-force telemetry is
 * logged at a fixed rate. On completion the samples are written to a
 * `ground_truth.csv` plus one self-contained SVG line chart per modality,
 * under `<output_dir>/<trajectory_name>/`.
 *
 * Node parameters:
 *   pidvid                (string, default "0403:6011")            — USB PID:VID to match.
 *   motor_id               (int,    default 0)                      — Target motor ID (0-14).
 *   output_dir              (string, default "/home/vscode/dev/trajectory_collection")
 *   max_connect_attempts (int,    default 4)                      — Scan/connect retries.
 *   connect_retry_delay_ms (int,    default 500)                    — Delay between retries.
 *   rate_hz                (double, default 100.0)                  — Motion/telemetry loop rate.
 *
 * Usage:
 *   ros2 run m80106_execs motion_data_collection --ros-args -p motor_id:=0
 */

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "m80106_lib/motor_driver.hpp"
#include "m80106_lib/motor_types.hpp"
#include "m80106_lib/multi_serial_go8_scanner.hpp"

namespace fs = std::filesystem;

namespace {

std::atomic<bool> g_quit{false};
void signalHandler(int) { g_quit.store(true); }

// ─────────────────────────────────────────────────────────────────────────────
// Telemetry sample
// ─────────────────────────────────────────────────────────────────────────────

struct TelemetrySample {
    double  t_s           = 0.0;   // relative time since motion start [s]
    float   pos_rotor     = 0.0f;
    float   pos_output    = 0.0f;
    float   speed_rotor   = 0.0f;
    float   speed_output  = 0.0f;
    float   torque_rotor  = 0.0f;
    float   torque_output = 0.0f;
    int     temp_c        = 0;
    int     error_code    = 0;
    uint8_t mode          = 0;
    int     foot_force    = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// String / prompt helpers
// ─────────────────────────────────────────────────────────────────────────────

std::string trim(const std::string & s) {
    const size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    const size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

std::string toLower(const std::string & s) {
    std::string out(s.size(), '\0');
    std::transform(s.begin(), s.end(), out.begin(),
                    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

/// Replace anything but [A-Za-z0-9_-] with '_'; falls back to a timestamped name if empty.
std::string sanitizeTrajectoryName(const std::string & raw) {
    std::string out;
    out.reserve(raw.size());
    for (char c : raw) {
        out += (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') ? c : '_';
    }
    if (out.empty()) {
        const auto secs = std::chrono::duration_cast<std::chrono::seconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
        out = "trajectory_" + std::to_string(secs);
    }
    return out;
}

double promptRotations() {
    while (true) {
        std::cout << "How many rotations would you like to perform? ";
        std::string line;
        if (!std::getline(std::cin, line)) return 1.0;
        line = trim(line);
        try {
            const double v = std::stod(line);
            if (v > 0.0) return v;
        } catch (const std::exception &) {
            // fall through to retry message
        }
        std::cout << "  Please enter a positive number.\n";
    }
}

std::string promptTrajectoryName() {
    std::cout << "Enter trajectory name: ";
    std::string line;
    std::getline(std::cin, line);
    return sanitizeTrajectoryName(trim(line));
}

/// Returns +1 for "right" (default) or -1 for "left".
int promptDirection() {
    std::cout << "Enter direction [left/right] (default: right): ";
    std::string line;
    std::getline(std::cin, line);
    const std::string dir = toLower(trim(line));
    if (dir.empty() || dir == "right" || dir == "r") return +1;
    if (dir == "left" || dir == "l") return -1;
    std::cout << "  Unrecognized direction '" << line << "', defaulting to right.\n";
    return +1;
}

double promptSpeed(double default_speed, double max_speed) {
    std::cout << "Enter rotation speed in rad/s, output-side (default: "
              << default_speed << "): ";
    std::string line;
    std::getline(std::cin, line);
    line = trim(line);
    if (line.empty()) return default_speed;
    try {
        const double v = std::stod(line);
        if (v <= 0.0) {
            std::cout << "  Speed must be positive; using default.\n";
            return default_speed;
        }
        if (v > max_speed) {
            std::cout << "  Clamping speed to max " << max_speed << " rad/s.\n";
            return max_speed;
        }
        return v;
    } catch (const std::exception &) {
        std::cout << "  Could not parse speed; using default.\n";
        return default_speed;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Filesystem helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Create `<base>/<name>`, appending `_1`, `_2`, ... if it already exists.
fs::path uniqueTrajectoryDir(const fs::path & base, const std::string & name) {
    fs::path candidate = base / name;
    int suffix = 1;
    while (fs::exists(candidate)) {
        candidate = base / (name + "_" + std::to_string(suffix));
        ++suffix;
    }
    fs::create_directories(candidate);
    return candidate;
}

// ─────────────────────────────────────────────────────────────────────────────
// Motor command helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Minimum velocity-damping floor (output-side 0.5) to avoid an undamped FOC command.
const float MIN_DAMPING_KW = m80106::toRotorKd(0.5f);

MotorCmd makeVelocityCmd(uint8_t id, float W_rotor, float K_W_rotor) {
    MotorCmd cmd;
    cmd.motorType = MotorType::GO_M8010_6;
    cmd.id = id;
    cmd.mode = m80106::toSDKMode(m80106::MotorMode::FOC);
    cmd.T = 0.0f;
    cmd.W = m80106::clamp(W_rotor, -m80106::PROTOCOL_MAX_SPEED_RADS, m80106::PROTOCOL_MAX_SPEED_RADS);
    cmd.Pos = 0.0f;
    cmd.K_P = 0.0f;
    cmd.K_W = std::max(m80106::clamp(K_W_rotor, 0.0f, m80106::MAX_KD), MIN_DAMPING_KW);
    return cmd;
}

/// Scan + brake-ping retry loop. Returns nullptr if the motor never confirms.
std::unique_ptr<m80106::MotorDriver> connectWithRetries(
    const rclcpp::Logger & logger,
    const std::string & pidvid,
    uint8_t target_id,
    int max_attempts,
    int delay_ms,
    std::string & out_port,
    std::string & out_hw_id)
{
    for (int attempt = 1; attempt <= max_attempts && rclcpp::ok(); ++attempt) {
        RCLCPP_INFO(logger, "Connection attempt %d/%d: scanning for motor ID %d ...",
                    attempt, max_attempts, static_cast<int>(target_id));

        const auto scan = m80106::scanAllPorts(pidvid);
        const auto all = scan.allMotors();
        const auto it = std::find_if(all.begin(), all.end(),
            [&](const m80106::DiscoveredMotor & m) { return m.id == target_id; });

        if (it != all.end()) {
            out_port = it->port;
            out_hw_id = it->hardware_id;
            try {
                auto driver = std::make_unique<m80106::MotorDriver>(out_port);
                MotorData fb;
                if (driver->brake(target_id, fb) && fb.correct) {
                    RCLCPP_INFO(logger, "Motor ID %d confirmed on %s (hw: %s).",
                                static_cast<int>(target_id), out_port.c_str(), out_hw_id.c_str());
                    return driver;
                }
                RCLCPP_WARN(logger, "  Found on scan but did not respond to a brake ping.");
            } catch (const std::exception & e) {
                RCLCPP_WARN(logger, "  Failed to open port %s: %s", out_port.c_str(), e.what());
            }
        } else {
            RCLCPP_WARN(logger, "  Motor ID %d not found in this scan.", static_cast<int>(target_id));
        }

        if (attempt < max_attempts) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }
    return nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
// CSV output
// ─────────────────────────────────────────────────────────────────────────────

void writeGroundTruthCsv(const fs::path & path, const std::vector<TelemetrySample> & samples) {
    std::ofstream f(path);
    f << "time_s,position_output_rad,position_rotor_rad,speed_output_rads,speed_rotor_rads,"
         "torque_output_nm,torque_rotor_nm,temperature_c,error_code,mode,foot_force\n";
    f << std::fixed << std::setprecision(6);
    for (const auto & s : samples) {
        f << s.t_s << ',' << s.pos_output << ',' << s.pos_rotor << ','
          << s.speed_output << ',' << s.speed_rotor << ','
          << s.torque_output << ',' << s.torque_rotor << ','
          << s.temp_c << ',' << s.error_code << ',' << static_cast<int>(s.mode) << ','
          << s.foot_force << '\n';
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Self-contained SVG line-chart generator (no external plotting dependency)
// ─────────────────────────────────────────────────────────────────────────────

std::string fmtTick(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", v);
    return buf;
}

void writeLineChartSvg(const fs::path & path,
                       const std::string & title,
                       const std::string & x_label,
                       const std::string & y_label,
                       const std::vector<double> & xs,
                       const std::vector<double> & ys)
{
    if (xs.empty() || ys.empty() || xs.size() != ys.size()) return;

    constexpr int width = 900, height = 450;
    constexpr int margin_left = 70, margin_right = 30, margin_top = 50, margin_bottom = 60;
    constexpr int plot_w = width - margin_left - margin_right;
    constexpr int plot_h = height - margin_top - margin_bottom;

    double xmin = *std::min_element(xs.begin(), xs.end());
    double xmax = *std::max_element(xs.begin(), xs.end());
    double ymin = *std::min_element(ys.begin(), ys.end());
    double ymax = *std::max_element(ys.begin(), ys.end());
    if (xmax - xmin < 1e-9) xmax = xmin + 1.0;
    if (ymax - ymin < 1e-6) { ymax += 1.0; ymin -= 1.0; }

    auto sx = [&](double x) { return margin_left + (x - xmin) / (xmax - xmin) * plot_w; };
    auto sy = [&](double y) { return margin_top + plot_h - (y - ymin) / (ymax - ymin) * plot_h; };

    std::ostringstream svg;
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
        << "\" height=\"" << height << "\" viewBox=\"0 0 " << width << " " << height << "\">\n";
    svg << "<rect width=\"" << width << "\" height=\"" << height << "\" fill=\"white\"/>\n";
    svg << "<text x=\"" << width / 2 << "\" y=\"25\" font-size=\"18\" font-family=\"sans-serif\" "
           "text-anchor=\"middle\" fill=\"black\">" << title << "</text>\n";

    svg << "<line x1=\"" << margin_left << "\" y1=\"" << margin_top
        << "\" x2=\"" << margin_left << "\" y2=\"" << margin_top + plot_h
        << "\" stroke=\"black\" stroke-width=\"1.5\"/>\n";
    svg << "<line x1=\"" << margin_left << "\" y1=\"" << margin_top + plot_h
        << "\" x2=\"" << margin_left + plot_w << "\" y2=\"" << margin_top + plot_h
        << "\" stroke=\"black\" stroke-width=\"1.5\"/>\n";

    constexpr int n_ticks = 5;
    for (int i = 0; i <= n_ticks; ++i) {
        const double xv = xmin + (xmax - xmin) * i / n_ticks;
        const double px = sx(xv);
        svg << "<line x1=\"" << px << "\" y1=\"" << margin_top + plot_h
            << "\" x2=\"" << px << "\" y2=\"" << margin_top + plot_h + 5
            << "\" stroke=\"black\"/>\n";
        svg << "<text x=\"" << px << "\" y=\"" << margin_top + plot_h + 20
            << "\" font-size=\"11\" text-anchor=\"middle\" font-family=\"sans-serif\">"
            << fmtTick(xv) << "</text>\n";

        const double yv = ymin + (ymax - ymin) * i / n_ticks;
        const double py = sy(yv);
        svg << "<line x1=\"" << margin_left - 5 << "\" y1=\"" << py
            << "\" x2=\"" << margin_left << "\" y2=\"" << py << "\" stroke=\"black\"/>\n";
        svg << "<text x=\"" << margin_left - 8 << "\" y=\"" << py + 4
            << "\" font-size=\"11\" text-anchor=\"end\" font-family=\"sans-serif\">"
            << fmtTick(yv) << "</text>\n";
    }

    svg << "<text x=\"" << margin_left + plot_w / 2 << "\" y=\"" << height - 15
        << "\" font-size=\"13\" text-anchor=\"middle\" font-family=\"sans-serif\">"
        << x_label << "</text>\n";
    svg << "<text x=\"15\" y=\"" << margin_top + plot_h / 2
        << "\" font-size=\"13\" text-anchor=\"middle\" font-family=\"sans-serif\" "
           "transform=\"rotate(-90 15 " << margin_top + plot_h / 2 << ")\">"
        << y_label << "</text>\n";

    svg << "<polyline fill=\"none\" stroke=\"#1f77b4\" stroke-width=\"2\" points=\"";
    for (size_t i = 0; i < xs.size(); ++i) {
        svg << sx(xs[i]) << "," << sy(ys[i]) << " ";
    }
    svg << "\"/>\n</svg>\n";

    std::ofstream f(path);
    f << svg.str();
}

void generateGraphs(const rclcpp::Logger & logger,
                    const fs::path & graphs_dir,
                    const std::vector<TelemetrySample> & samples)
{
    if (samples.empty()) {
        RCLCPP_WARN(logger, "No samples collected; skipping graph generation.");
        return;
    }
    fs::create_directories(graphs_dir);

    std::vector<double> t, pos, speed, torque, temp, foot;
    t.reserve(samples.size());
    pos.reserve(samples.size());
    speed.reserve(samples.size());
    torque.reserve(samples.size());
    temp.reserve(samples.size());
    foot.reserve(samples.size());
    for (const auto & s : samples) {
        t.push_back(s.t_s);
        pos.push_back(static_cast<double>(s.pos_output));
        speed.push_back(static_cast<double>(s.speed_output));
        torque.push_back(static_cast<double>(s.torque_output));
        temp.push_back(static_cast<double>(s.temp_c));
        foot.push_back(static_cast<double>(s.foot_force));
    }

    writeLineChartSvg(graphs_dir / "position_output_rad.svg",
                      "Output Position vs Time", "Time (s)", "Position (rad)", t, pos);
    writeLineChartSvg(graphs_dir / "speed_output_rads.svg",
                      "Output Speed vs Time", "Time (s)", "Speed (rad/s)", t, speed);
    writeLineChartSvg(graphs_dir / "torque_output_nm.svg",
                      "Output Torque vs Time", "Time (s)", "Torque (N*m)", t, torque);
    writeLineChartSvg(graphs_dir / "temperature_c.svg",
                      "Temperature vs Time", "Time (s)", "Temperature (C)", t, temp);
    writeLineChartSvg(graphs_dir / "foot_force.svg",
                      "Foot Force vs Time", "Time (s)", "Foot Force (ADC)", t, foot);
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("motion_data_collection");
    const auto logger = node->get_logger();

    node->declare_parameter<std::string>("pidvid", "0403:6011");
    node->declare_parameter<int>("motor_id", 0);
    node->declare_parameter<std::string>("output_dir", "/home/vscode/dev/trajectory_collection");
    node->declare_parameter<int>("max_connect_attempts", 4);
    node->declare_parameter<int>("connect_retry_delay_ms", 500);
    node->declare_parameter<double>("rate_hz", 100.0);

    const std::string pidvid = node->get_parameter("pidvid").as_string();
    const int motor_id_param = node->get_parameter("motor_id").as_int();
    const std::string output_dir = node->get_parameter("output_dir").as_string();
    const int max_connect_attempts = node->get_parameter("max_connect_attempts").as_int();
    const int connect_retry_delay_ms = node->get_parameter("connect_retry_delay_ms").as_int();
    const double rate_hz = node->get_parameter("rate_hz").as_double();

    if (motor_id_param < 0 || motor_id_param > static_cast<int>(m80106::MAX_MOTOR_ID)) {
        RCLCPP_ERROR(logger, "motor_id=%d is out of range [0, %d]. Aborting.",
                     motor_id_param, static_cast<int>(m80106::MAX_MOTOR_ID));
        rclcpp::shutdown();
        return 1;
    }
    const uint8_t motor_id = static_cast<uint8_t>(motor_id_param);

    // ── Connect (with retries) ────────────────────────────────────────────
    std::string port, hw_id;
    auto driver = connectWithRetries(logger, pidvid, motor_id, max_connect_attempts,
                                     connect_retry_delay_ms, port, hw_id);
    if (!driver) {
        RCLCPP_ERROR(logger, "Could not confirm motor ID %d after %d attempt(s). Aborting.",
                     static_cast<int>(motor_id), max_connect_attempts);
        rclcpp::shutdown();
        return 1;
    }

    // ── Interactive prompts ───────────────────────────────────────────────
    const double rotations = promptRotations();
    const std::string trajectory_name = promptTrajectoryName();
    const int direction = promptDirection();
    const double speed_output = promptSpeed(5.0, static_cast<double>(m80106::OUTPUT_MAX_SPEED_RADS));

    RCLCPP_INFO(logger,
                "Starting collection: %.3f rotation(s), direction=%s, speed=%.2f rad/s (output), "
                "trajectory='%s'",
                rotations, direction > 0 ? "right" : "left", speed_output, trajectory_name.c_str());

    // ── Read starting position ────────────────────────────────────────────
    float start_pos_rotor = 0.0f;
    bool have_start_pos = false;
    for (int i = 0; i < 5 && rclcpp::ok(); ++i) {
        MotorData fb;
        if (driver->brake(motor_id, fb) && fb.correct) {
            start_pos_rotor = fb.Pos;
            have_start_pos = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!have_start_pos) {
        RCLCPP_ERROR(logger, "Could not read an initial position from the motor. Aborting.");
        rclcpp::shutdown();
        return 1;
    }

    const float target_pos_rotor = start_pos_rotor +
        static_cast<float>(direction) * m80106::toRotorPos(static_cast<float>(rotations * 2.0 * M_PI));

    const float K_W = m80106::toRotorKd(2.0f);
    const float W_rotor = static_cast<float>(direction) * m80106::toRotorSpeed(static_cast<float>(speed_output));
    const MotorCmd motion_cmd = makeVelocityCmd(motor_id, W_rotor, K_W);

    // ── Motion + telemetry collection loop ────────────────────────────────
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const double expected_time_s = (rotations * 2.0 * M_PI) / speed_output;
    const double timeout_s = 3.0 * expected_time_s + 5.0;
    const auto period = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / rate_hz));

    std::vector<TelemetrySample> samples;
    const auto motion_start = std::chrono::steady_clock::now();
    bool reached = false;
    bool hw_error = false;
    bool timed_out = false;

    RCLCPP_INFO(logger, "Motion started (target ~%.1fs, timeout %.1fs) ...",
                expected_time_s, timeout_s);

    while (rclcpp::ok() && !g_quit.load()) {
        const auto tick_start = std::chrono::steady_clock::now();
        const double elapsed_s = std::chrono::duration<double>(tick_start - motion_start).count();

        MotorCmd c = motion_cmd;
        MotorData fb;
        if (driver->sendRecv(c, fb) && fb.correct) {
            TelemetrySample s;
            s.t_s = elapsed_s;
            s.pos_rotor = fb.Pos;
            s.pos_output = m80106::toOutputPos(fb.Pos);
            s.speed_rotor = fb.W;
            s.speed_output = m80106::toOutputSpeed(fb.W);
            s.torque_rotor = fb.T;
            s.torque_output = fb.T * m80106::GEAR_RATIO;
            s.temp_c = fb.Temp;
            s.error_code = fb.MError;
            s.mode = fb.mode;
            s.foot_force = fb.footForce;
            samples.push_back(s);

            if (m80106::toMotorError(fb.MError) != m80106::MotorError::NONE) {
                RCLCPP_ERROR(logger, "Hardware error during motion: %s (code %d). Stopping.",
                             m80106::errorString(m80106::toMotorError(fb.MError)), fb.MError);
                hw_error = true;
                break;
            }

            if ((direction > 0 && fb.Pos >= target_pos_rotor) ||
                (direction < 0 && fb.Pos <= target_pos_rotor)) {
                reached = true;
                break;
            }
        }

        if (elapsed_s > timeout_s) {
            RCLCPP_WARN(logger, "Timeout reached before completing the requested rotations.");
            timed_out = true;
            break;
        }

        const auto tick_elapsed = std::chrono::steady_clock::now() - tick_start;
        const auto remaining = period - std::chrono::duration_cast<std::chrono::microseconds>(tick_elapsed);
        if (remaining.count() > 0) {
            std::this_thread::sleep_for(remaining);
        }
    }

    if (g_quit.load()) {
        RCLCPP_WARN(logger, "Motion aborted by signal.");
    }

    // ── Brake and settle ──────────────────────────────────────────────────
    for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
        MotorData fb;
        driver->brake(motor_id, fb);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    RCLCPP_INFO(logger, "Motion finished: %zu sample(s) collected (%s).",
                samples.size(),
                reached ? "target reached" : hw_error ? "hardware error" :
                timed_out ? "timed out" : "aborted");

    // ── Write outputs ─────────────────────────────────────────────────────
    if (!samples.empty()) {
        const fs::path traj_dir = uniqueTrajectoryDir(fs::path(output_dir), trajectory_name);
        const fs::path csv_path = traj_dir / "ground_truth.csv";
        writeGroundTruthCsv(csv_path, samples);
        generateGraphs(logger, traj_dir / "graphs", samples);

        RCLCPP_INFO(logger, "Wrote %s", csv_path.c_str());
        RCLCPP_INFO(logger, "Wrote graphs under %s", (traj_dir / "graphs").c_str());
    } else {
        RCLCPP_WARN(logger, "No telemetry collected; nothing written to disk.");
    }

    rclcpp::shutdown();
    return (hw_error || (!reached && samples.empty())) ? 1 : 0;
}
