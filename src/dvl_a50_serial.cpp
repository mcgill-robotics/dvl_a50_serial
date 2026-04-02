#include "dvl_a50_serial/dvl_a50_serial.hpp"
#include <iostream>
#include <chrono>

namespace dvl_a50_serial {

DvlA50Serial::DvlA50Serial() : running_(false), wait_for_ack_(false), ack_received_(false), nak_received_(false) {}

DvlA50Serial::~DvlA50Serial() {
    disconnect();
}

bool DvlA50Serial::connect(const std::string& port, int baud_rate) {
    if (!port_.open(port, baud_rate)) {
        return false;
    }
    running_ = true;
    read_thread_ = std::thread(&DvlA50Serial::read_loop, this);
    return true;
}

void DvlA50Serial::disconnect() {
    running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    port_.close();
}

void DvlA50Serial::set_velocity_callback(std::function<void(const VelocityReport&)> cb) {
    velocity_cb_ = cb;
}

void DvlA50Serial::set_dead_reckoning_callback(std::function<void(const DeadReckoningReport&)> cb) {
    dead_reckoning_cb_ = cb;
}

void DvlA50Serial::set_transducer_callback(std::function<void(const TransducerReport&)> cb) {
    transducer_cb_ = cb;
}

void DvlA50Serial::set_error_callback(std::function<void(const std::string&)> cb) {
    error_cb_ = cb;
}

bool DvlA50Serial::send_command(const std::string& cmd, int timeout_ms) {
    std::lock_guard<std::mutex> lock(ack_mutex_);
    
    ack_received_ = false;
    nak_received_ = false;
    wait_for_ack_ = true;

    if (!port_.write_line(cmd)) {
        wait_for_ack_ = false;
        return false;
    }

    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < timeout_ms) {
        if (ack_received_) {
            wait_for_ack_ = false;
            return true;
        }
        if (nak_received_) {
            wait_for_ack_ = false;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    wait_for_ack_ = false;
    return false; // timeout
}

bool DvlA50Serial::configure(int speed_of_sound, bool acoustic_enabled, bool led_enabled, int mounting_rotation_offset, const std::string& range_mode, int timeout_ms) {
    // wcs,[speed_of_sound],[mounting_rotation_offset],[acoustic_enabled],[dark_mode_enabled],[range_mode],[periodic_cycling_enabled]
    std::string cmd = "wcs," + std::to_string(speed_of_sound) + "," + 
                      std::to_string(mounting_rotation_offset) + "," + 
                      (acoustic_enabled ? "y" : "n") + "," + 
                      (!led_enabled ? "y" : "n") + "," + // led_enabled=false means dark_mode_enabled=y
                      range_mode + ",y"; // Assuming periodic_cycling_enabled is y by default based on spec
    return send_command(cmd, timeout_ms);
}

bool DvlA50Serial::reset_dead_reckoning(int timeout_ms) {
    return send_command("wcr", timeout_ms);
}

bool DvlA50Serial::calibrate_gyro(int timeout_ms) {
    return send_command("wcg", timeout_ms);
}

bool DvlA50Serial::trigger_ping(int timeout_ms) {
    return send_command("wcx", timeout_ms);
}

bool DvlA50Serial::set_protocol(int protocol_number, int timeout_ms) {
    return send_command("wcp," + std::to_string(protocol_number), timeout_ms);
}

void DvlA50Serial::read_loop() {
    while (running_) {
        std::string line = port_.read_line(100); // 100 ms timeout
        if (line.empty()) continue;

        auto result = DvlParser::parse(line);
        if (!result.is_valid) {
            if (!result.error.empty() && error_cb_) {
                error_cb_(result.error);
            }
            continue;
        }

        if (result.command == "wrz") {
            auto rep = DvlParser::parse_wrz(result.args);
            if (rep && velocity_cb_) {
                velocity_cb_(*rep);
            }
        } else if (result.command == "wrp") {
            auto rep = DvlParser::parse_wrp(result.args);
            if (rep && dead_reckoning_cb_) {
                dead_reckoning_cb_(*rep);
            }
        } else if (result.command == "wru") {
            auto rep = DvlParser::parse_wru(result.args);
            if (rep && transducer_cb_) {
                transducer_cb_(*rep);
            }
        } else if (result.command == "wra") {
            if (wait_for_ack_) {
                ack_received_ = true;
            }
        } else if (result.command == "wrn" || result.command == "wr?" || result.command == "wr!") {
            if (wait_for_ack_) {
                nak_received_ = true;
            }
        }
    }
}

} // namespace dvl_a50_serial
