#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <termios.h>
#include <sys/select.h>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <chrono>
#include <queue>
#include <array>
#include <mutex> // 引入互斥锁

class SerialPort
{
public:
    using SharedPtr = std::shared_ptr<SerialPort>;

    SerialPort(std::string port, speed_t baudrate, int timeout_ms = 2)
    {
        set_timeout(timeout_ms);
        Init(port, baudrate);
    }

    ~SerialPort()
    {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    /**
     * @brief 发送数据（线程安全）
     */
    ssize_t send(const uint8_t* data, size_t len)
    {
        std::lock_guard<std::mutex> lock(mtx_); // 加锁
        if (fd_ < 0) return -1;
        ssize_t ret = ::write(fd_, data, len);
        return ret;
    }

    /**
     * @brief 基础接收数据（内部使用，带超时）
     */
    ssize_t recv_basic(uint8_t* data, size_t len)
    {
        // 注意：select 会修改 timeout 结构体，每次需重新设置
        struct timeval tv = timeout_config_; 
        fd_set rSet;
        FD_ZERO(&rSet);
        FD_SET(fd_, &rSet);

        ssize_t recv_len = 0;
        int res = select(fd_ + 1, &rSet, NULL, NULL, &tv);

        if (res > 0) {
            recv_len = ::read(fd_, data, len);
        }
        return recv_len;
    }

    /**
     * @brief 协议解析接收（线程安全）
     * @param data 存储解析后数据的数组
     * @param head 帧头字节
     * @param len  期望的整帧长度
     */
    void recv(uint8_t* data, uint8_t head, ssize_t len)
    {
        if (len <= 0) return;

        std::lock_guard<std::mutex> lock(mtx_); // 加锁保护所有成员变量（队列和缓冲区）

        // 1. 从硬件读取新数据存入队列
        ssize_t actual_read = recv_basic(recv_buf_.data(), recv_buf_.size());
        if (actual_read > 0) {
            for (int i = 0; i < actual_read; i++) {
                recv_queue_.push(recv_buf_[i]);
            }
        }

        // 2. 查找帧头
        // 增加对队列长度的检查，防止 pop 空队列
        while (recv_queue_.size() >= (size_t)len)
        {
            if (recv_queue_.front() != head)
            {
                recv_queue_.pop();
                continue;
            }
            break;
        }

        // 3. 提取完整帧
        if (recv_queue_.size() >= (size_t)len)
        {
            for (int i = 0; i < len; i++)
            {
                data[i] = recv_queue_.front();
                recv_queue_.pop();
            }
        }
    }

    void set_timeout(int timeout_ms)
    {
        timeout_config_.tv_sec = timeout_ms / 1000;
        timeout_config_.tv_usec = (timeout_ms % 1000) * 1000;
    }

private:
    void Init(std::string port, speed_t baudrate)
    {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0)
        {
            printf("[Serial] Open %s failed\n", port.c_str());
            exit(-1);
        }

        struct termios option;
        memset(&option, 0, sizeof(option));
        tcgetattr(fd_, &option);

        option.c_oflag = 0;
        option.c_lflag = 0;
        option.c_iflag = 0;

        cfsetispeed(&option, baudrate);
        cfsetospeed(&option, baudrate);

        option.c_cflag &= ~CSIZE;
        option.c_cflag |= CS8;      // 8位数据
        option.c_cflag &= ~PARENB;  // 无校验
        option.c_iflag &= ~INPCK;
        option.c_cflag &= ~CSTOPB;  // 1位停止位

        option.c_cc[VTIME] = 0;
        option.c_cc[VMIN] = 0;

        tcflush(fd_, TCIFLUSH);
        if (tcsetattr(fd_, TCSANOW, &option) != 0) {
            printf("[Serial] Set attributes failed\n");
        }
    }

    int fd_;
    struct timeval timeout_config_;
    
    std::mutex mtx_;                         // 线程锁
    std::queue<uint8_t> recv_queue_;         // 接收队列
    std::array<uint8_t, 2048> recv_buf_;     // 临时缓冲区（增大到2048）
};

#endif // SERIAL_PORT_H