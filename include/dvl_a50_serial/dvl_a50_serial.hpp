#pragma once

#include "dvl_a50_serial/serial_port.hpp"
#include "dvl_a50_serial/dvl_parser.hpp"
#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>

namespace dvl_a50_serial {

class DvlA50Serial {
public:
    DvlA50Serial();
    ~DvlA50Serial();

    bool connect(const std::string& port, int baud_rate = 115200);
    void disconnect();

    void set_velocity_callback(std::function<void(const VelocityReport&)> cb);
    void set_dead_reckoning_callback(std::function<void(const DeadReckoningReport&)> cb);
    void set_transducer_callback(std::function<void(const TransducerReport&)> cb);
    void set_error_callback(std::function<void(const std::string&)> cb);

    bool send_command(const std::string& cmd, int timeout_ms = 15000); // 15s timeout because gyro calib can take up to 15s
    bool write_command_to_port(const std::string& cmd);
    bool wait_for_ack(int timeout_ms);
    bool wait_for_config(int timeout_ms);

    bool configure(int speed_of_sound, bool acoustic_enabled, bool led_enabled, int mounting_rotation_offset, const std::string& range_mode, int timeout_ms);
    bool reset_dead_reckoning(int timeout_ms);
    bool calibrate_gyro(int timeout_ms);
    bool trigger_ping(int timeout_ms);
    bool set_protocol(int protocol_number, int timeout_ms);
    bool query_current_config(int timeout_ms);
    DVLConfiguration get_current_config();
private:
    void read_loop();

    SerialPort port_;
    DVLConfiguration current_config_;
    std::atomic<bool> running_;
    std::thread read_thread_;

    std::function<void(const VelocityReport&)> velocity_cb_;
    std::function<void(const DeadReckoningReport&)> dead_reckoning_cb_;
    std::function<void(const TransducerReport&)> transducer_cb_;
    std::function<void(const std::string&)> error_cb_;

    std::mutex ack_mutex_;
    std::atomic<bool> wait_for_ack_;
    std::atomic<bool> ack_received_;
    std::atomic<bool> nak_received_;
    std::atomic<bool> config_updated_;
};

} // namespace dvl_a50_serial
