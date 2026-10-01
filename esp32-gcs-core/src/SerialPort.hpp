#ifndef SERIALPORT_HPP
#define SERIALPORT_HPP

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <errno.h>
#endif

class SerialPort {
private:
#ifdef _WIN32
    HANDLE hSerial;
#else
    int fd;
#endif
    bool connected;

public:
    SerialPort(const char* portName) {
        connected = false;

#ifdef _WIN32
        hSerial = CreateFileA(portName,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL);

        if (hSerial == INVALID_HANDLE_VALUE) {
            return;
        }

        DCB dcbSerialParams = { 0 };
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

        if (!GetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            return;
        }

        dcbSerialParams.BaudRate = CBR_115200;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity   = NOPARITY;

        // Mod binar, fara flow control
        dcbSerialParams.fBinary = TRUE;
        dcbSerialParams.fNull = FALSE;
        dcbSerialParams.fOutX = FALSE;
        dcbSerialParams.fInX = FALSE;

        if (!SetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            return;
        }

        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout         = MAXDWORD;
        timeouts.ReadTotalTimeoutConstant    = 0;
        timeouts.ReadTotalTimeoutMultiplier  = 0;
        timeouts.WriteTotalTimeoutConstant   = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;
        SetCommTimeouts(hSerial, &timeouts);
        connected = true;
#else
        fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0) {
            return;
        }

        // Non-blocking mode
        fcntl(fd, F_SETFL, FNDELAY);

        struct termios options;
        memset(&options, 0, sizeof(options));
        if (tcgetattr(fd, &options) != 0) {
            close(fd);
            fd = -1;
            return;
        }

        // Baud rate: 115200
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);

        // 8N1 (8 bits, no parity, 1 stop bit)
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;

        // No hardware flow control
        #ifdef CRTSCTS
        options.c_cflag &= ~CRTSCTS;
        #endif

        options.c_cflag |= (CLOCAL | CREAD);

        // Raw input
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        // Raw output
        options.c_oflag &= ~OPOST;

        // No software flow control
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

        // Non-blocking read (return immediately if no data)
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        tcflush(fd, TCIFLUSH);
        if (tcsetattr(fd, TCSANOW, &options) != 0) {
            close(fd);
            fd = -1;
            return;
        }

        connected = true;
#endif
    }

    ~SerialPort() {
        if (connected) {
#ifdef _WIN32
            CloseHandle(hSerial);
#else
            if (fd >= 0) {
                close(fd);
                fd = -1;
            }
#endif
        }
    }

    bool writeData(const std::string& data) {
        if (!connected) return false;
#ifdef _WIN32
        DWORD bytesWritten;
        return WriteFile(hSerial, data.c_str(), static_cast<DWORD>(data.length()), &bytesWritten, NULL);
#else
        if (fd < 0) return false;
        ssize_t written = write(fd, data.c_str(), data.length());
        return (written >= 0);
#endif
    }

    std::vector<uint8_t> readBytes() {
        if (!connected) return {};
        uint8_t buffer[512];
#ifdef _WIN32
        DWORD bytesRead = 0;
        if (ReadFile(hSerial, buffer, sizeof(buffer), &bytesRead, NULL) && bytesRead > 0) {
            return std::vector<uint8_t>(buffer, buffer + bytesRead);
        }
#else
        if (fd < 0) return {};
        ssize_t bytesRead = read(fd, buffer, sizeof(buffer));
        if (bytesRead > 0) {
            return std::vector<uint8_t>(buffer, buffer + bytesRead);
        }
#endif
        return {};
    }

    int readData(char* buffer, unsigned int nbChar) {
        if (!connected) return 0;

#ifdef _WIN32
        DWORD errors;
        COMSTAT status;
        DWORD bytesRead = 0;
        unsigned int toRead = 0;

        ClearCommError(hSerial, &errors, &status);

        if (status.cbInQue > 0) {
            if (status.cbInQue > nbChar) {
                toRead = nbChar;
            } else {
                toRead = status.cbInQue;
            }

            if (ReadFile(hSerial, buffer, toRead, &bytesRead, NULL)) {
                return static_cast<int>(bytesRead);
            }
        }
        return 0;
#else
        if (fd < 0) return 0;
        int bytesAvailable = 0;
        if (ioctl(fd, FIONREAD, &bytesAvailable) < 0) {
            return 0;
        }

        if (bytesAvailable > 0) {
            size_t toRead = (bytesAvailable > static_cast<int>(nbChar)) ? nbChar : static_cast<size_t>(bytesAvailable);
            ssize_t bytesRead = read(fd, buffer, toRead);
            if (bytesRead > 0) {
                return static_cast<int>(bytesRead);
            }
        }
        return 0;
#endif
    }

    bool isConnected() const {
        return connected;
    }

    bool writeDataBytes(const char* buffer, unsigned int bufSize) {
        if (!connected) return false;
#ifdef _WIN32
        DWORD bytesWritten;
        return WriteFile(hSerial, buffer, bufSize, &bytesWritten, NULL);
#else
        if (fd < 0) return false;
        ssize_t written = write(fd, buffer, bufSize);
        return (written >= 0);
#endif
    }
};

#endif // SERIALPORT_HPP