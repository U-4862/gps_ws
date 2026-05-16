#include "usart.hpp"
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <cerrno>

SerialPort::SerialPort(std::string device, speed_t baud_rate,
               int vmin, int vtime_ds) 
               : device_(std::move(device)),baud_rate_(baud_rate),
                vmin_(vmin),vtime_ds_(vtime_ds){}

SerialPort::~SerialPort(){
    closePort();
}

bool SerialPort::openPort()
{
        std::lock_guard<std::mutex> lock(mutex_);

        if (fd_ >= 0)
        {
            return true;
        }

        fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
        {
            last_error_ = "open(" + device_ + ") failed: " + std::string(std::strerror(errno));
            return false;
        }

        termios tty {};
        if (tcgetattr(fd_, &tty) != 0)
        {
            last_error_ = "tcgetattr(" + device_ + ") failed: " + std::string(std::strerror(errno));
            closeUnlocked();
            return false;
        }

        cfsetospeed(&tty, baud_rate_);
        cfsetispeed(&tty, baud_rate_);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
        tty.c_iflag &= ~(INPCK | ISTRIP | PARMRK);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = static_cast<cc_t>(vmin_);
        tty.c_cc[VTIME] = static_cast<cc_t>(vtime_ds_);

        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        {
            last_error_ = "tcsetattr(" + device_ + ") failed: " + std::string(std::strerror(errno));
            closeUnlocked();
            return false;
        }

        if (tcflush(fd_, TCIOFLUSH) != 0)
        {
            last_error_ = "tcflush(" + device_ + ") failed: " + std::string(std::strerror(errno));
            closeUnlocked();
            return false;
        }

        last_error_.clear();
        return true;
}


void SerialPort::closePort()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closeUnlocked();
    }

bool SerialPort::isOpen() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return fd_ >= 0;
    }

std::string SerialPort::lastError() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_error_;
    }


bool SerialPort::writeExact(const void* data, std::size_t len)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (fd_ < 0)
        {
            last_error_ = "writeExact called on closed port: " + device_;
            return false;
        }

        const auto* bytes = static_cast<const std::uint8_t*>(data);
        std::size_t total_written = 0;

        while (total_written < len)
        {
            const ssize_t written = ::write(fd_, bytes + total_written, len - total_written);
            if (written < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }

                last_error_ = "write(" + device_ + ") failed: " + std::string(std::strerror(errno));
                return false;
            }

            total_written += static_cast<std::size_t>(written);
        }

        if (tcdrain(fd_) != 0)
        {
            last_error_ = "tcdrain(" + device_ + ") failed: " + std::string(std::strerror(errno));
            return false;
        }

        last_error_.clear();
        return true;
    }


ssize_t SerialPort::readSome(void* buffer, std::size_t max_len)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (fd_ < 0)
        {
            last_error_ = "readSome called on closed port: " + device_;
            return -1;
        }

        const ssize_t n = ::read(fd_, buffer, max_len);
        if (n < 0)
        {
            if (errno == EINTR)
            {
                return 0;
            }

            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                return 0;
            }

            last_error_ = "read(" + device_ + ") failed: " + std::string(std::strerror(errno));
            return -1;
        }

        last_error_.clear();
        return n;
    }

    void SerialPort::closeUnlocked()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }



    