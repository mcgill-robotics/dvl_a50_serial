#pragma once

#include <string>
#include <vector>
#include <optional>
#include <array>
#include <cstdint>

namespace dvl_a50_serial {

struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct TransducerReport {
    int id = -1;
    double velocity = 0.0;
    double distance = -1.0;
    int rssi = -999;
    int nsd = -999;
};

struct VelocityReport {
    Vector3 velocity;
    bool valid = false;
    double altitude = -1.0;
    double fom = 0.0;
    std::array<double, 9> covariance{0.0};
    std::array<TransducerReport, 4> transducers;
    uint64_t time_of_validity = 0;
    uint64_t time_of_transmission = 0;
    double time = 0.0;
    int status = 0;
};

struct DeadReckoningReport {
    double time_stamp = 0.0;
    Vector3 position;
    double pos_std = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    int status = 0;
};

struct DVLConfiguration {
    int speed_of_sound;
    bool acoustic_enabled;
    bool dark_mode_enabled;
    int mounting_rotation_offset;
    std::string range_mode;
};

class DvlParser {
public:
    DvlParser() = default;
    
    struct ParseResult {
        bool is_valid = false;
        std::string command;
        std::vector<std::string> args;
        std::string error;
    };

    static uint8_t crc8(const uint8_t* message, size_t length);
    static ParseResult parse(const std::string& line);

    static std::optional<VelocityReport> parse_wrz(const std::vector<std::string>& args);
    static std::optional<DeadReckoningReport> parse_wrp(const std::vector<std::string>& args);
    static std::optional<TransducerReport> parse_wru(const std::vector<std::string>& args);
    static std::optional<DVLConfiguration> parse_wrc(const std::vector<std::string>& args);
};

} // namespace dvl_a50_serial
