#pragma once

#include <termios.h>
#include <sys/select.h>
#include <array>
#include <cerrno>
#include <cstdio>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <memory>
#include <queue>
#include <stdexcept>

class SerialPort {
public:
    using SharedPtr = std::shared_ptr<SerialPort>;

    SerialPort(std::string port, speed_t baudrate, int timeout_ms = 2) {
        set_timeout(timeout_ms);
        init(port, baudrate);
    }

    ~SerialPort() {
        if(fd_ >= 0) {
            close(fd_);
        }
    }

    ssize_t send(const uint8_t* data, size_t len) {
        if(fd_ < 0) return -1;
        // tcflush(fd_, TCIFLUSH);
        ssize_t ret = ::write(fd_, data, len);
        // tcdrain(fd_);
        return ret;
    }

    ssize_t recv(uint8_t* data, size_t len) {
        if(fd_ < 0) return -1;
        fd_set rset;
        FD_ZERO(&rset);
        FD_SET(fd_, &rset);

        timeval tv = timeout_;
        ssize_t recv_len = 0;

        int ret = select(fd_ + 1, &rset, nullptr, nullptr, &tv);
        if(ret > 0) {
            recv_len = ::read(fd_, data, len);
        }
        return recv_len;
    }

    bool recv_frame(uint8_t* data, uint8_t head, ssize_t len) {
        ssize_t recv_len = recv(recv_buf.data(), len);
        for(int i = 0; i < recv_len; ++i) {
            recv_queue.push(recv_buf[i]);
        }

        while(recv_queue.size() >= static_cast<size_t>(len)) {
            if(recv_queue.front() != head) {
                recv_queue.pop();
                continue;
            }

            for(int i = 0; i < len; ++i) {
                data[i] = recv_queue.front();
                recv_queue.pop();
            }
            return true;
        }
        return false;
    }

    void set_timeout(int timeout_ms) {
        timeout_.tv_sec = timeout_ms / 1000;
        timeout_.tv_usec = (timeout_ms % 1000) * 1000;
    }

private:
    void init(std::string port, speed_t baudrate) {
        int ret;

        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
        if(fd_ < 0) {
            throw std::runtime_error("Open serial port " + port + " failed: " + std::string(strerror(errno)));
        }

        struct termios option;
        memset(&option, 0, sizeof(option));

        ret = tcgetattr(fd_, &option);
        if(ret != 0) {
            perror("tcgetattr");
        }

        option.c_oflag = 0;
        option.c_lflag = 0;
        option.c_iflag = 0;

        cfsetispeed(&option, baudrate);
        cfsetospeed(&option, baudrate);

        option.c_cflag &= ~CSIZE;
        option.c_cflag |= CS8;
        option.c_cflag &= ~PARENB;
        option.c_iflag &= ~INPCK;
        option.c_cflag &= ~CSTOPB;
        option.c_cflag |= (CLOCAL | CREAD);

        option.c_cc[VTIME] = 0;
        option.c_cc[VMIN] = 0;
        option.c_lflag |= CBAUDEX;

        ret = tcflush(fd_, TCIFLUSH);
        if(ret != 0) {
            perror("tcflush");
        }

        ret = tcsetattr(fd_, TCSANOW, &option);
        if(ret != 0) {
            perror("tcsetattr");
        }
    }

    int fd_{ -1 };
    timeval timeout_{};

    std::queue<uint8_t> recv_queue;
    std::array<uint8_t, 1024> recv_buf;
};
