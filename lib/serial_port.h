#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <string>
#include <vector>
#include <stdint.h>
#include <asm/termbits.h>
#include <fcntl.h>
#include <unistd.h>     // UNIX standard function definitions
#include <system_error> // For throwing std::system_error
#include <sys/ioctl.h>  // Used for TCGETS2, which is required for custom baud rates
#include <cassert>
#include <string.h>
#include <iostream>
#include <algorithm>

// Threat Message Header
const unsigned int HEADER_LOCKED_ON_EMITTER         = 0x49;
const unsigned int HEADER_PRELOCKED_EMITTER         = 0x4A;
const unsigned int HEADER_NEXT_MESSAGE              = 0x40;

// Bit Message Header
const unsigned int HEADER_BIT_STATUS                = 0x4B;

// Library Load Header
const unsigned int HEADER_LIBRARY_STATUS            = 0x4C;

// EMP Header
const unsigned int HEADER_EMP                       = 0xFE;

// Normal Mode Message Header
const unsigned int HEADER_ENGINE_MONITOR_MESSAGE    = 0xF0;

// IBIT Message Header
const unsigned int HEADER_ENTER_TEST_MODE           = 0xF3;
const unsigned int HEADER_IBIT_ERROR_CODE           = 0xF5;
const unsigned int TERMINATOR_EXIT_TEST_MODE        = 0xF6;

// Bingo Set Message Header
const unsigned int HEADER_ENTER_BINGO_SET_MODE      = 0xF9;
const unsigned int HEADER_EXIT_BINGO_SET_MODE       = 0xFA;

// Terminator For All
const unsigned int TERMINATOR                       = 0x80;

enum class BaudRate
{
    B_0,
    B_50,
    B_75,
    B_110,
    B_134,
    B_150,
    B_200,
    B_300,
    B_600,
    B_1200,
    B_1800,
    B_2400,
    B_4800,
    B_9600,
    B_19200,
    B_38400,
    B_57600,
    B_115200,
    B_230400,
    B_460800,
    B_CUSTOM, // Placeholder
};

enum class NumDataBits
{
    FIVE,
    SIX,
    SEVEN,
    EIGHT,
};

enum class Parity
{
    NONE,
    EVEN,
    ODD,
};

enum class NumStopBits
{
    ONE,
    TWO,
};

enum class State
{
    CLOSED,
    OPEN,
};

enum class HardwareFlowControl
{
    OFF,
    ON,
};

enum class SoftwareFlowControl
{
    OFF,
    ON,
};

enum class BaudRateType
{
    STANDARD,
    CUSTOM,
};

class SerialPort
{
private:
    void ConfigureTermios();
    termios2 GetTermios2();
    void SetTermios2(termios2 tty);
    State state_;
    std::string device_;
    BaudRateType baudRateType_;
    BaudRate baudRateStandard_;
    speed_t baudRateCustom_;
    NumDataBits numDataBits_ = NumDataBits::EIGHT;
    Parity parity_ = Parity::NONE;
    NumStopBits numStopBits_ = NumStopBits::ONE;
    HardwareFlowControl hardwareFlowControl_ = HardwareFlowControl::OFF;
    SoftwareFlowControl softwareFlowControl_ = SoftwareFlowControl::OFF;
    int fileDesc_;
    bool echo_;
    int32_t timeout_ms_;
    //uint8_t bufferTemp[255];
    std::vector<uint8_t> bufferTemp;
    std::vector<uint8_t> parsedData;
    //std::vector<char> readBuffer_;
    uint8_t readBuffer_[255];
    uint8_t data[255];
    uint8_t readBufferSize_B_;

    static constexpr BaudRate defaultBaudRate_ = BaudRate::B_9600;
    static constexpr int32_t defaultTimeout_ms_ = -1;
    static constexpr unsigned char defaultReadBufferSize_B_ = 255;

public:
    SerialPort();
    SerialPort(const std::string &device, BaudRate baudRate);
    SerialPort(const std::string &device, BaudRate baudRate, NumDataBits numDataBits, Parity parity, NumStopBits numStopBits);
    SerialPort(const std::string &device, BaudRate baudRate, NumDataBits numDataBits, Parity parity,
               NumStopBits numStopBits, HardwareFlowControl hardwareFlowControl, SoftwareFlowControl softwareFlowControl);
    SerialPort(const std::string &device, speed_t baudRate);
    virtual ~SerialPort();
    void SetDevice(const std::string &device);
    void SetBaudRate(BaudRate baudRate);
    void SetNumDataBits(NumDataBits numDataBits);
    void SetParity(Parity parity);
    void SetNumStopBits(NumStopBits numStopBits);
    void SetTimeout(int32_t timeout_ms);
    void SetEcho(bool value);
    void Open();
    void Close();
    void Write(const std::string &data);
    void WriteBinary(const std::vector<uint8_t> &data);
    void Read(std::string &data);
    void ReadBinary(std::vector<uint8_t> &data);
    int32_t Available();
    State GetState();
};

#endif