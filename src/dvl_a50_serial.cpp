#include "dvl_a50_serial/dvl_a50_serial.hpp"
#include <iostream>
#include <chrono>

namespace dvl_a50_serial {

DvlA50Serial::DvlA50Serial() : running_(false), wait_for_ack_(false), ack_received_(false), nak_received_(false), config_updated_(false) {}

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
    if (!write_command_to_port(cmd)) {
        return false;
    }
    return wait_for_ack(timeout_ms);
}

bool DvlA50Serial::write_command_to_port(const std::string& cmd) {
    // Generate CRC-8 checksum explicitly
    std::string full_cmd = cmd;
    uint8_t crc = DvlParser::crc8(reinterpret_cast<const uint8_t*>(full_cmd.data()), full_cmd.size());
    char hex[4];
    snprintf(hex, sizeof(hex), "*%02x", crc);
    full_cmd += hex;
    std::lock_guard<std::mutex> lock(ack_mutex_);
    
    ack_received_ = false;
    nak_received_ = false;

    if (!port_.write_line(full_cmd)) {
        wait_for_ack_ = false;
        return false;
    }
    return true;
}

bool DvlA50Serial::wait_for_ack(int timeout_ms) {
    ack_received_ = false;
    nak_received_ = false;
    wait_for_ack_ = true;
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

bool DvlA50Serial::wait_for_config(int timeout_ms) {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() < timeout_ms) {
        if (config_updated_) {
            return true;
        }
        if (nak_received_) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false; // timeout
}

bool DvlA50Serial::configure(const DVLConfiguration& config, int timeout_ms) {
    // wcs,[speed_of_sound],[mounting_rotation_offset],[acoustic_enabled],[dark_mode_enabled],[range_mode],[periodic_cycling_enabled]
    std::string cmd = "wcs," + 
                    std::to_string(config.speed_of_sound) + "," + 
                    std::to_string(config.mounting_rotation_offset) + "," + 
                    (config.acoustic_enabled ? "y" : "n") + "," + 
                    (config.dark_mode_enabled ? "y" : "n") + "," +
                    config.range_mode + "," +
                    (config.periodic_cycling_enabled ? "y" : "n");
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

bool DvlA50Serial::query_current_config(int timeout_ms) {
    // stale config flag since it might be updated
    config_updated_ = false;
    if (!write_command_to_port("wcc")) {
        return false;
    }
    return wait_for_config(timeout_ms);
}

DVLConfiguration DvlA50Serial::get_current_config() {
    return current_config_;
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
            std::cout << "[DVL_ACK] Valid standard ACK parsed natively" << std::endl;
            if (wait_for_ack_) {
                ack_received_ = true;
            }
        } else if (result.command == "wrc") {
            auto rep = DvlParser::parse_wrc(result.args);
            if (rep) {
                current_config_ = *rep;
                std::cout << "[DVL_CONFIG] Obtained latest configuration from device" << std::endl;
                config_updated_ = true;
            }
        } 
        else if (result.command == "wrn" || result.command == "wr?" || result.command == "wr!") {
            std::cout << "[DVL_NAK] Hardware rejected command: " << result.command << std::endl;
            if (wait_for_ack_) {
                nak_received_ = true;
            }
        } 
    }
}

} // namespace dvl_a50_serial
