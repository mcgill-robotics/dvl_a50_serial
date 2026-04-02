#include "dvl_a50_serial/serial_port.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <iostream>
#include <vector>

namespace dvl_a50_serial {

SerialPort::SerialPort() : fd_(-1) {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, int baud_rate) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ == -1) {
        return false;
    }

    if (tcflush(fd_, TCIOFLUSH)) {
        // Warning, not fatal
    }

    struct termios options;
    if (tcgetattr(fd_, &options)) {
        close();
        return false;
    }

    cfmakeraw(&options);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);

    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;

    switch (baud_rate) {
        case 4800:   cfsetospeed(&options, B4800);   break;
        case 9600:   cfsetospeed(&options, B9600);   break;
        case 19200:  cfsetospeed(&options, B19200);  break;
        case 38400:  cfsetospeed(&options, B38400);  break;
        case 115200: cfsetospeed(&options, B115200); break;
        default:
            cfsetospeed(&options, B115200);
            break;
    }

    cfsetispeed(&options, cfgetospeed(&options));

    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &options)) {
        close();
        return false;
    }

    return true;
}

void SerialPort::close() {
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
}

std::string SerialPort::read_line(int timeout_ms) {
    if (fd_ == -1) return "";

    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;

    while (true) {
        size_t newline_pos = read_buffer_.find('\n');
        if (newline_pos != std::string::npos) {
            std::string line = read_buffer_.substr(0, newline_pos);
            read_buffer_.erase(0, newline_pos + 1);
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            return line;
        }

        int ret = poll(&pfd, 1, timeout_ms);
        if (ret > 0 && (pfd.revents & POLLIN)) {
            char chunk[256];
            ssize_t n = ::read(fd_, chunk, sizeof(chunk));
            if (n > 0) {
                read_buffer_.append(chunk, n);
            } else {
                break;
            }
        } else {
            break;
        }
    }

    return "";
}

bool SerialPort::write_line(const std::string& msg) {
    if (fd_ == -1) return false;
    
    std::string to_write = msg;
    if (to_write.empty() || to_write.back() != '\n') {
        to_write += '\n'; // According to doc it ends with \n or \r\n or \r
    }

    size_t written = 0;
    while (written < to_write.size()) {
        ssize_t n = ::write(fd_, to_write.c_str() + written, to_write.size() - written);
        if (n < 0) {
            return false;
        }
        written += n;
    }
    return true;
}

} // namespace dvl_a50_serial
