#pragma once

#include <string>

namespace dvl_a50_serial {

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    bool open(const std::string& port, int baud_rate = 115200);
    void close();
    std::string read_line(int timeout_ms = 100);
    bool write_line(const std::string& msg);

private:
    int fd_;
    std::string read_buffer_;
};

} // namespace dvl_a50_serial
