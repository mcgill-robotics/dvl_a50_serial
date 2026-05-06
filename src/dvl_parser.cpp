#include "dvl_a50_serial/dvl_parser.hpp"
#include <charconv>
#include <iostream>
#include <sstream>

namespace dvl_a50_serial {

static const uint8_t lookup_table[256] = {
    0x00U,0x07U,0x0EU,0x09U,0x1CU,0x1BU,0x12U,0x15U,
    0x38U,0x3FU,0x36U,0x31U,0x24U,0x23U,0x2AU,0x2DU,
    0x70U,0x77U,0x7EU,0x79U,0x6CU,0x6BU,0x62U,0x65U,
    0x48U,0x4FU,0x46U,0x41U,0x54U,0x53U,0x5AU,0x5DU,
    0xE0U,0xE7U,0xEEU,0xE9U,0xFCU,0xFBU,0xF2U,0xF5U,
    0xD8U,0xDFU,0xD6U,0xD1U,0xC4U,0xC3U,0xCAU,0xCDU,
    0x90U,0x97U,0x9EU,0x99U,0x8CU,0x8BU,0x82U,0x85U,
    0xA8U,0xAFU,0xA6U,0xA1U,0xB4U,0xB3U,0xBAU,0xBDU,
    0xC7U,0xC0U,0xC9U,0xCEU,0xDBU,0xDCU,0xD5U,0xD2U,
    0xFFU,0xF8U,0xF1U,0xF6U,0xE3U,0xE4U,0xEDU,0xEAU,
    0xB7U,0xB0U,0xB9U,0xBEU,0xABU,0xACU,0xA5U,0xA2U,
    0x8FU,0x88U,0x81U,0x86U,0x93U,0x94U,0x9DU,0x9AU,
    0x27U,0x20U,0x29U,0x2EU,0x3BU,0x3CU,0x35U,0x32U,
    0x1FU,0x18U,0x11U,0x16U,0x03U,0x04U,0x0DU,0x0AU,
    0x57U,0x50U,0x59U,0x5EU,0x4BU,0x4CU,0x45U,0x42U,
    0x6FU,0x68U,0x61U,0x66U,0x73U,0x74U,0x7DU,0x7AU,
    0x89U,0x8EU,0x87U,0x80U,0x95U,0x92U,0x9BU,0x9CU,
    0xB1U,0xB6U,0xBFU,0xB8U,0xADU,0xAAU,0xA3U,0xA4U,
    0xF9U,0xFEU,0xF7U,0xF0U,0xE5U,0xE2U,0xEBU,0xECU,
    0xC1U,0xC6U,0xCFU,0xC8U,0xDDU,0xDAU,0xD3U,0xD4U,
    0x69U,0x6EU,0x67U,0x60U,0x75U,0x72U,0x7BU,0x7CU,
    0x51U,0x56U,0x5FU,0x58U,0x4DU,0x4AU,0x43U,0x44U,
    0x19U,0x1EU,0x17U,0x10U,0x05U,0x02U,0x0BU,0x0CU,
    0x21U,0x26U,0x2FU,0x28U,0x3DU,0x3AU,0x33U,0x34U,
    0x4EU,0x49U,0x40U,0x47U,0x52U,0x55U,0x5CU,0x5BU,
    0x76U,0x71U,0x78U,0x7FU,0x6AU,0x6DU,0x64U,0x63U,
    0x3EU,0x39U,0x30U,0x37U,0x22U,0x25U,0x2CU,0x2BU,
    0x06U,0x01U,0x08U,0x0FU,0x1AU,0x1DU,0x14U,0x13U,
    0xAEU,0xA9U,0xA0U,0xA7U,0xB2U,0xB5U,0xBCU,0xBBU,
    0x96U,0x91U,0x98U,0x9FU,0x8AU,0x8DU,0x84U,0x83U,
    0xDEU,0xD9U,0xD0U,0xD7U,0xC2U,0xC5U,0xCCU,0xCBU,
    0xE6U,0xE1U,0xE8U,0xEFU,0xFAU,0xFDU,0xF4U,0xF3U,
};

uint8_t DvlParser::crc8(const uint8_t* message, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum = lookup_table[message[i] ^ checksum];
    }
    return checksum;
}

DvlParser::ParseResult DvlParser::parse(const std::string& line) {
    ParseResult result;
    if (line.empty() || line[0] != 'w') {
        return result;
    }

    // Find the checksum separator
    size_t star_pos = line.find_last_of('*');
    std::string payload;

    if (star_pos != std::string::npos && star_pos + 2 < line.size()) {
        // verify checksum
        std::string hex_str = line.substr(star_pos + 1, 2);
        while (!hex_str.empty() && (hex_str.back() == '\r' || hex_str.back() == '\n')) {
            hex_str.pop_back();
        }
        
        if (hex_str.size() == 2) {
            uint8_t expected_crc = 0;
            auto [ptr, ec] = std::from_chars(hex_str.data(), hex_str.data() + hex_str.size(), expected_crc, 16);
            if (ec == std::errc()) {
                uint8_t calculated_crc = crc8(reinterpret_cast<const uint8_t*>(line.data()), star_pos);
                if (expected_crc != calculated_crc) {
                    result.error = "CRC Mismatch";
                    return result;
                }
            }
        }
        payload = line.substr(0, star_pos);
    } else {
        payload = line;
        while (!payload.empty() && (payload.back() == '\r' || payload.back() == '\n')) {
            payload.pop_back();
        }
    }

    result.is_valid = true;
    size_t first_comma = payload.find(',');
    if (first_comma == std::string::npos) {
        result.command = payload;
        return result;
    }
    result.command = payload.substr(0, first_comma);
    
    std::string args_str = payload.substr(first_comma + 1);
    
    // Split by ',' or ';'
    size_t start = 0;
    while (start < args_str.size()) {
        size_t end = args_str.find_first_of(",;", start);
        if (end == std::string::npos) {
            result.args.push_back(args_str.substr(start));
            break;
        } else {
            result.args.push_back(args_str.substr(start, end - start));
            start = end + 1;
        }
    }

    return result;
}

static double safe_double(const std::string& str) {
    if (str.empty()) return 0.0;
    try { return std::stod(str); } catch (...) { return 0.0; }
}

static uint64_t safe_u64(const std::string& str) {
    if (str.empty()) return 0ULL;
    try { return std::stoull(str); } catch (...) { return 0ULL; }
}

static int safe_int(const std::string& str) {
    if (str.empty()) return 0;
    try { return std::stoi(str, nullptr, 16); } catch(...) {
        try { return std::stoi(str); } catch(...) { return 0; }
    }
}

std::optional<VelocityReport> DvlParser::parse_wrz(const std::vector<std::string>& args) {
    if (args.size() != 19 && args.size() != 11) {
        return std::nullopt;
    }

    VelocityReport rep;
    rep.velocity.x = safe_double(args[0]);
    rep.velocity.y = safe_double(args[1]);
    rep.velocity.z = safe_double(args[2]);
    rep.valid = (args[3] == "y");
    rep.altitude = safe_double(args[4]);
    rep.fom = safe_double(args[5]);

    int offset = 0;
    if (args.size() == 19) {
        for (int i = 0; i < 9; ++i) {
            rep.covariance[i] = safe_double(args[6 + i]);
        }
        offset = 9;
    } else if (args.size() == 11) {
        // When invalid, the covariance is mathematically truncated structurally
        rep.covariance[0] = safe_double(args[6]);
        offset = 1;
    }

    rep.time_of_validity = safe_u64(args[6 + offset]);
    rep.time_of_transmission = safe_u64(args[6 + offset + 1]);
    rep.time = safe_double(args[6 + offset + 2]);
    rep.status = safe_int(args[6 + offset + 3]);

    return rep;
}

std::optional<DeadReckoningReport> DvlParser::parse_wrp(const std::vector<std::string>& args) {
    if (args.size() != 9) return std::nullopt;

    DeadReckoningReport rep;
    rep.time_stamp = safe_double(args[0]);
    rep.position.x = safe_double(args[1]);
    rep.position.y = safe_double(args[2]);
    rep.position.z = safe_double(args[3]);
    rep.pos_std = safe_double(args[4]);
    rep.roll = safe_double(args[5]);
    rep.pitch = safe_double(args[6]);
    rep.yaw = safe_double(args[7]);
    rep.status = safe_int(args[8]);
    
    return rep;
}

std::optional<TransducerReport> DvlParser::parse_wru(const std::vector<std::string>& args) {
    if (args.size() != 5) return std::nullopt;

    TransducerReport rep;
    rep.id = safe_int(args[0]);
    rep.velocity = safe_double(args[1]);
    rep.distance = safe_double(args[2]);
    rep.rssi = safe_int(args[3]);
    rep.nsd = safe_int(args[4]);

    return rep;
}

std::optional<DVLConfiguration> DvlParser::parse_wrc(const std::vector<std::string>& args) {
    if (args.size() != 6) return std::nullopt;

    DVLConfiguration config;
    config.speed_of_sound = safe_int(args[0]);
    config.acoustic_enabled = (args[1] == "y");
    config.dark_mode_enabled = (args[2] == "y");
    config.mounting_rotation_offset = safe_int(args[3]);
    config.range_mode = args[4];

    return config;
}

} // namespace dvl_a50_serial
