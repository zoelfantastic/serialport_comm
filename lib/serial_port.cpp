#include "serial_port.h"
#include "exception.h"

void SerialPort::ConfigureTermios()
{
    //================== CONFIGURE ==================//

    // termios tty = GetTermios();
    termios2 tty = GetTermios2();

    //================= (.c_cflag) ===============//

    // Set num. data bits
    // See https://man7.org/linux/man-pages/man3/tcflush.3.html
    tty.c_cflag &= ~CSIZE; // CSIZE is a mask for the number of bits per character
    switch (numDataBits_)
    {
    case NumDataBits::FIVE:
        tty.c_cflag |= CS5;
        break;
    case NumDataBits::SIX:
        tty.c_cflag |= CS6;
        break;
    case NumDataBits::SEVEN:
        tty.c_cflag |= CS7;
        break;
    case NumDataBits::EIGHT:
        tty.c_cflag |= CS8;
        break;
    default:
        THROW_EXCEPT("numDataBits_ value not supported!");
    }

    // Set parity
    // See https://man7.org/linux/man-pages/man3/tcflush.3.html
    switch (parity_)
    {
    case Parity::NONE:
        tty.c_cflag &= ~PARENB;
        break;
    case Parity::EVEN:
        tty.c_cflag |= PARENB;
        tty.c_cflag &= ~PARODD; // Clearing PARODD makes the parity even
        break;
    case Parity::ODD:
        tty.c_cflag |= PARENB;
        tty.c_cflag |= PARODD;
        break;
    default:
        THROW_EXCEPT("parity_ value not supported!");
    }

    // Set num. stop bits
    switch (numStopBits_)
    {
    case NumStopBits::ONE:
        tty.c_cflag &= ~CSTOPB;
        break;
    case NumStopBits::TWO:
        tty.c_cflag |= CSTOPB;
        break;
    default:
        THROW_EXCEPT("numStopBits_ value not supported!");
    }

    // Configure flow control
    switch (hardwareFlowControl_)
    {
    case HardwareFlowControl::OFF:
        tty.c_cflag &= ~CRTSCTS;
        break;

    case HardwareFlowControl::ON: // Hardware flow control (RTS/CTS)
        tty.c_cflag |= CRTSCTS;
        break;

    default:
        // We should never get here.
        THROW_EXCEPT("flowControl_ value not supported!");
        break;
    }

    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    //===================== BAUD RATE =================//

    // We used to use cfsetispeed() and cfsetospeed() with the B... macros, but this didn't allow
    // us to set custom baud rates. So now to support both standard and custom baud rates lets
    // just make everything "custom". This giant switch statement could be replaced with a map/lookup
    // in the future
    if (baudRateType_ == BaudRateType::STANDARD)
    {
        tty.c_cflag &= ~CBAUD;
        tty.c_cflag |= CBAUDEX;
        switch (baudRateStandard_)
        {
        case BaudRate::B_0:
            // cfsetispeed(&tty, B0);
            // cfsetospeed(&tty, B0);
            tty.c_ispeed = 0;
            tty.c_ospeed = 0;
            break;
        case BaudRate::B_50:
            // cfsetispeed(&tty, B50);
            // cfsetospeed(&tty, B50);
            tty.c_ispeed = 50;
            tty.c_ospeed = 50;
            break;
        case BaudRate::B_75:
            // cfsetispeed(&tty, B75);
            // cfsetospeed(&tty, B75);
            tty.c_ispeed = 75;
            tty.c_ospeed = 75;
            break;
        case BaudRate::B_110:
            // cfsetispeed(&tty, B110);
            // cfsetospeed(&tty, B110);
            tty.c_ispeed = 110;
            tty.c_ospeed = 110;
            break;
        case BaudRate::B_134:
            // cfsetispeed(&tty, B134);
            // cfsetospeed(&tty, B134);
            tty.c_ispeed = 134;
            tty.c_ospeed = 134;
            break;
        case BaudRate::B_150:
            // cfsetispeed(&tty, B150);
            // cfsetospeed(&tty, B150);
            tty.c_ispeed = 150;
            tty.c_ospeed = 150;
            break;
        case BaudRate::B_200:
            // cfsetispeed(&tty, B200);
            // cfsetospeed(&tty, B200);
            tty.c_ispeed = 200;
            tty.c_ospeed = 200;
            break;
        case BaudRate::B_300:
            // cfsetispeed(&tty, B300);
            // cfsetospeed(&tty, B300);
            tty.c_ispeed = 300;
            tty.c_ospeed = 300;
            break;
        case BaudRate::B_600:
            // cfsetispeed(&tty, B600);
            // cfsetospeed(&tty, B600);
            tty.c_ispeed = 600;
            tty.c_ospeed = 600;
            break;
        case BaudRate::B_1200:
            // cfsetispeed(&tty, B1200);
            // cfsetospeed(&tty, B1200);
            tty.c_ispeed = 1200;
            tty.c_ospeed = 1200;
            break;
        case BaudRate::B_1800:
            // cfsetispeed(&tty, B1800);
            // cfsetospeed(&tty, B1800);
            tty.c_ispeed = 1800;
            tty.c_ospeed = 1800;
            break;
        case BaudRate::B_2400:
            // cfsetispeed(&tty, B2400);
            // cfsetospeed(&tty, B2400);
            tty.c_ispeed = 2400;
            tty.c_ospeed = 2400;
            break;
        case BaudRate::B_4800:
            // cfsetispeed(&tty, B4800);
            // cfsetospeed(&tty, B4800);
            tty.c_ispeed = 4800;
            tty.c_ospeed = 4800;
            break;
        case BaudRate::B_9600:
            // cfsetispeed(&tty, B9600);
            // cfsetospeed(&tty, B9600);
            tty.c_ispeed = 9600;
            tty.c_ospeed = 9600;
            break;
        case BaudRate::B_19200:
            // cfsetispeed(&tty, B19200);
            // cfsetospeed(&tty, B19200);
            tty.c_ispeed = 19200;
            tty.c_ospeed = 19200;
            break;
        case BaudRate::B_38400:
            // cfsetispeed(&tty, B38400);
            // cfsetospeed(&tty, B38400);
            tty.c_ispeed = 38400;
            tty.c_ospeed = 38400;
            break;
        case BaudRate::B_57600:
            // cfsetispeed(&tty, B57600);
            // cfsetospeed(&tty, B57600);
            tty.c_ispeed = 57600;
            tty.c_ospeed = 57600;
            break;
        case BaudRate::B_115200:
            // cfsetispeed(&tty, B115200);
            // cfsetospeed(&tty, B115200);
            tty.c_ispeed = 115200;
            tty.c_ospeed = 115200;
            break;
        case BaudRate::B_230400:
            // cfsetispeed(&tty, B230400);
            // cfsetospeed(&tty, B230400);
            tty.c_ispeed = 230400;
            tty.c_ospeed = 230400;
            break;
        case BaudRate::B_460800:
            // cfsetispeed(&tty, B460800);
            // cfsetospeed(&tty, B460800);
            tty.c_ispeed = 460800;
            tty.c_ospeed = 460800;
            break;
        default:
            throw std::runtime_error(std::string() + "baudRate passed to " + __PRETTY_FUNCTION__ + " unrecognized.");
        }
    }
    // This does no different than STANDARD atm, but let's keep
    // them separate for now....
    else if (baudRateType_ == BaudRateType::CUSTOM)
    {
        tty.c_cflag &= ~CBAUD;
        tty.c_cflag |= CBAUDEX;
        // tty.c_cflag |= BOTHER;
        tty.c_ispeed = baudRateCustom_;
        tty.c_ospeed = baudRateCustom_;

        // #include <linux/serial.h>
        // // configure port to use custom speed instead of 38400
        // struct serial_struct ss;
        // ioctl(fileDesc_, TIOCGSERIAL, &ss);
        // ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
        // ss.custom_divisor = (ss.baud_base + (baudRateCustom_ / 2)) / baudRateCustom_;
        // int closestSpeed = ss.baud_base / ss.custom_divisor;

        // if (closestSpeed < baudRateCustom_ * 98 / 100 || closestSpeed > baudRateCustom_ * 102 / 100) {
        // 	printf("Cannot set serial port speed to %d. Closest possible is %d\n", baudRateCustom_, closestSpeed);
        // }

        // ioctl(fileDesc_, TIOCSSERIAL, &ss);

        // cfsetispeed(&tty, B38400);
        // cfsetospeed(&tty, B38400);
    }
    else
    {
        // Should never get here, bug in this libraries code!
        assert(false);
    }

    //===================== (.c_oflag) =================//

    tty.c_oflag = 0;       // No remapping, no delays
    tty.c_oflag &= ~OPOST; // Make raw

    //================= CONTROL CHARACTERS (.c_cc[]) ==================//

    // c_cc[VTIME] sets the inter-character timer, in units of 0.1s.
    // Only meaningful when port is set to non-canonical mode
    // VMIN = 0, VTIME = 0: No blocking, return immediately with what is available
    // VMIN > 0, VTIME = 0: read() waits for VMIN bytes, could block indefinitely
    // VMIN = 0, VTIME > 0: Block until any amount of data is available, OR timeout occurs
    // VMIN > 0, VTIME > 0: Block until either VMIN characters have been received, or VTIME
    //                      after first character has elapsed
    // c_cc[WMIN] sets the number of characters to block (wait) for when read() is called.
    // Set to 0 if you don't want read to block. Only meaningful when port set to non-canonical mode

    if (timeout_ms_ == -1)
    {
        // Always wait for at least one byte, this could
        // block indefinitely
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 1;
    }
    else if (timeout_ms_ == 0)
    {
        // Setting both to 0 will give a non-blocking read
        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;
    }
    else if (timeout_ms_ > 0)
    {
        tty.c_cc[VTIME] = (cc_t)(timeout_ms_ / 100); // 0.5 seconds read timeout
        tty.c_cc[VMIN] = 0;
    }

    //======================== (.c_iflag) ====================//

    switch (softwareFlowControl_)
    {
    case SoftwareFlowControl::OFF:
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        break;

    case SoftwareFlowControl::ON:
        tty.c_iflag |= (IXON | IXOFF | IXANY);
        break;
    }

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    //=========================== LOCAL MODES (c_lflag) =======================//

    // Canonical input is when read waits for EOL or EOF characters before returning. In non-canonical mode, the rate at which
    // read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]
    tty.c_lflag &= ~ICANON; // Turn off canonical input, which is suitable for pass-through
    // Configure echo depending on echo_ boolean
    if (echo_)
    {
        tty.c_lflag |= ECHO;
    }
    else
    {
        tty.c_lflag &= ~(ECHO);
    }
    tty.c_lflag &= ~ECHOE;  // Turn off echo erase (echo erase only relevant if canonical input is active)
    tty.c_lflag &= ~ECHONL; //
    tty.c_lflag &= ~ISIG;   // Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters

    // Try and use raw function call
    // cfmakeraw(&tty);

    // this->SetTermios(tty);
    this->SetTermios2(tty);

    /*
    // Flush port, then apply attributes
    tcflush(this->fileDesc, TCIFLUSH);

    if(tcsetattr(this->fileDesc, TCSANOW, &tty) != 0)
    {
        // Error occurred
        this->sp->PrintError(SmartPrint::Ss() << "Could not apply terminal attributes for \"" << this->filePath << "\" - " << strerror(errno));
        return;
    }*/
}

termios2 SerialPort::GetTermios2()
{
    struct termios2 term2;

    ioctl(fileDesc_, TCGETS2, &term2);

    return term2;

    // term2.c_cflag &= ~CBAUD;  /* Remove current BAUD rate */
    // term2.c_cflag |= BOTHER;  /* Allow custom BAUD rate using int input */
    // term2.c_ispeed = speed;   /* Set the input BAUD rate */
    // term2.c_ospeed = speed;   /* Set the output BAUD rate */

    // ioctl(fd, TCSETS2, &term2);
}

void SerialPort::SetTermios2(termios2 tty)
{
    ioctl(fileDesc_, TCSETS2, &tty);
}

/// @brief   Default constructor. You must specify at least the device before calling Open().
SerialPort::SerialPort()
{
    echo_ = false;
    timeout_ms_ = defaultTimeout_ms_;
    baudRateType_ = BaudRateType::STANDARD;
    baudRateStandard_ = defaultBaudRate_;
    readBufferSize_B_ = defaultReadBufferSize_B_;
    int arraySize = 0;

    currMessageState = NO_MESSAGE_HEADER;

    header = NULL;
    terminator = NULL;
    endOfData = NULL;
    state_ = State::CLOSED;
}

SerialPort::SerialPort(const std::string &device, BaudRate baudRate) : SerialPort()
{
    device_ = device;
    baudRateType_ = BaudRateType::STANDARD;
    baudRateStandard_ = baudRate;
}

SerialPort::SerialPort(const std::string &device, BaudRate baudRate, NumDataBits numDataBits, Parity parity, NumStopBits numStopBits) : SerialPort()
{
    device_ = device;
    baudRateType_ = BaudRateType::STANDARD;
    baudRateStandard_ = baudRate;
    numDataBits_ = numDataBits;
    parity_ = parity;
    numStopBits_ = numStopBits;
}

SerialPort::SerialPort(const std::string &device, BaudRate baudRate, NumDataBits numDataBits, Parity parity, NumStopBits numStopBits,
                       HardwareFlowControl hardwareFlowControl, SoftwareFlowControl softwareFlowControl) : SerialPort()
{
    device_ = device;
    baudRateType_ = BaudRateType::STANDARD;
    baudRateStandard_ = baudRate;
    numDataBits_ = numDataBits;
    parity_ = parity;
    numStopBits_ = numStopBits;
    hardwareFlowControl_ = hardwareFlowControl;
    softwareFlowControl_ = softwareFlowControl;
}

SerialPort::SerialPort(const std::string &device, speed_t baudRate) : SerialPort()
{
    device_ = device;
    baudRateType_ = BaudRateType::CUSTOM;
    baudRateCustom_ = baudRate;
}

SerialPort::~SerialPort()
{
    try
    {
        Close();
    }
    catch (...)
    {
        // We can't do anything about this!
        // But we don't want to throw within destructor, so swallow
    }
}

void SerialPort::SetDevice(const std::string &device)
{
    device_ = device;
    if (state_ == State::OPEN)
    {
        ConfigureTermios();
    }
}

void SerialPort::SetBaudRate(BaudRate baudRate)
{
    baudRateType_ = BaudRateType::STANDARD;
    baudRateStandard_ = baudRate;
    if (state_ == State::OPEN)
        ConfigureTermios();
}

//  Call this to set the num. of data bits.
void SerialPort::SetNumDataBits(NumDataBits numDataBits)
{
    numDataBits_ = numDataBits;
    if (state_ == State::OPEN)
        ConfigureTermios();
}

void SerialPort::SetParity(Parity parity)
{
    parity_ = parity;
    if (state_ == State::OPEN)
        ConfigureTermios();
}

//  Call this to set the number of stop bits.
void SerialPort::SetNumStopBits(NumStopBits numStopBits)
{
    numStopBits_ = numStopBits;
    if (state_ == State::OPEN)
        ConfigureTermios();
}

/// \brief      Sets the read timeout (in milliseconds)/blocking mode.
/// \details    Only call when state != OPEN. This method manupulates VMIN and VTIME.
/// \param      timeout_ms  Set to -1 to infinite timeout, 0 to return immediately with any data (non
///             blocking, or >0 to wait for data for a specified number of milliseconds). Timeout will
///             be rounded to the nearest 100ms (a Linux API restriction). Maximum value limited to
///             25500ms (another Linux API restriction).
void SerialPort::SetTimeout(int32_t timeout_ms)
{
    if (timeout_ms < -1)
        THROW_EXCEPT(std::string() + "timeout_ms provided to " + __PRETTY_FUNCTION__ + " was < -1, which is invalid.");
    if (timeout_ms > 25500)
        THROW_EXCEPT(std::string() + "timeout_ms provided to " + __PRETTY_FUNCTION__ + " was > 25500, which is invalid.");
    if (state_ == State::OPEN)
        THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called while state == OPEN.");
    timeout_ms_ = timeout_ms;
}

void SerialPort::SetEcho(bool value)
{
    echo_ = value;
    ConfigureTermios();
}

void SerialPort::Open()
{
    if (device_.empty())
    {
        THROW_EXCEPT("Attempted to open file when file path has not been assigned to.");
    }

    // Attempt to open file
    // this->fileDesc = open(this->filePath, O_RDWR | O_NOCTTY | O_NDELAY);

    // O_RDONLY for read-only, O_WRONLY for write only, O_RDWR for both read/write access
    // 3rd, optional parameter is mode_t mode
    fileDesc_ = open(device_.c_str(), O_RDWR);

    // Check status
    if (fileDesc_ == -1)
    {
        THROW_EXCEPT("Could not open device \"" + device_ + "\". Is the device name correct and do you have read/write permissions?");
    }

    ConfigureTermios();

    // std::cout << "COM port opened successfully." << std::endl;
    state_ = State::OPEN;
}

void SerialPort::Close()
{
    if (fileDesc_ != -1)
    {
        auto retVal = close(fileDesc_);
        if (retVal != 0)
            THROW_EXCEPT("Tried to close serial port " + device_ + ", but close() failed.");

        fileDesc_ = -1;
    }

    state_ = State::CLOSED;
}

void SerialPort::Write(const std::string &data)
{
    if (state_ != State::OPEN)
        THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");

    if (fileDesc_ < 0)
    {
        THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but file descriptor < 0, indicating file has not been opened.");
    }

    int writeResult = write(fileDesc_, data.c_str(), data.size());

    // Check status
    if (writeResult == -1)
    {
        throw std::system_error(EFAULT, std::system_category());
    }
}

void SerialPort::WriteBinary(const std::vector<uint8_t> &data)
{
    if (state_ != State::OPEN)
        THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");

    if (fileDesc_ < 0)
    {
        THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but file descriptor < 0, indicating file has not been opened.");
    }

    int writeResult = write(fileDesc_, data.data(), data.size());

    // Check status
    if (writeResult == -1)
    {
        throw std::system_error(EFAULT, std::system_category());
    }
}

void SerialPort::Read(std::string &data)
{
    if (fileDesc_ == 0)
    {
        // this->sp->PrintError(SmartPrint::Ss() << "Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
        // return false;
        THROW_EXCEPT("Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
    }

    // Read from file
    // We provide the underlying raw array from the readBuffer_ vector to this C api.
    // This will work because we do not delete/resize the vector while this method
    // is called
    ssize_t n = read(fileDesc_, &readBuffer_[0], readBufferSize_B_);

    // Error Handling
    if (n < 0)
    {
        // Read was unsuccessful
        throw std::system_error(EFAULT, std::system_category());
    }
    else if (n == 0)
    {
        // n == 0 means EOS, but also returned on device disconnection. We try to get termios2 to distinguish two these two states
        struct termios2 term2;
        int rv = ioctl(fileDesc_, TCGETS2, &term2);

        if (rv != 0)
        {
            throw std::system_error(EFAULT, std::system_category());
        }
    }
    else if (n > 0)
    {
        // data += std::string(&readBuffer_[0], n);
    }

    // If code reaches here, read must of been successful
}

void SerialPort::ReadBinary(std::vector<uint8_t> &data)
{

    if (fileDesc_ == 0)
    {
        // this->sp->PrintError(SmartPrint::Ss() << "Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
        // return false;
        THROW_EXCEPT("Read() was called but file descriptor (fileDesc) was 0, indicating file has not been opened.");
    }

    // Read from file
    // We provide the underlying raw array from the readBuffer_ vector to this C api.
    // This will work because we do not delete/resize the vector while this method
    // is called
    ssize_t n = read(fileDesc_, readBuffer_, readBufferSize_B_);
    // Error Handling
    if (n < 0)
    {
        // Read was unsuccessful
        throw std::system_error(EFAULT, std::system_category());
    }
    else if (n == 0)
    {
        // n == 0 means EOS, but also returned on device disconnection. We try to get termios2 to distinguish two these two states
        struct termios2 term2;
        int rv = ioctl(fileDesc_, TCGETS2, &term2);

        if (rv != 0)
        {
            throw std::system_error(EFAULT, std::system_category());
        }
    }
    else if (n > 0)
    {
        if (arraySize >= MAX_SIZE)
        {
            // buffer full
            printf("buffer full\n");
        }
        else
        {
           
            memcpy(&bufferTemp[arraySize], readBuffer_, n);
            arraySize += n;
            // printf("array size init: %d\n", arraySize);;
            // printf("===BUFFER DATA===\n");
            // printf("%d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d -%d\n",
            //        bufferTemp[0], bufferTemp[1], bufferTemp[2], bufferTemp[3], bufferTemp[4], bufferTemp[5],
            //        bufferTemp[6], bufferTemp[7], bufferTemp[8], bufferTemp[9], bufferTemp[10], bufferTemp[11],
            //        bufferTemp[12], bufferTemp[13], bufferTemp[14], bufferTemp[15], bufferTemp[16], bufferTemp[17],
            //        bufferTemp[18], bufferTemp[19], bufferTemp[20]);

            for (int i = 0; i < arraySize; i++)
            {
                switch (currMessageState)
                {
                case NO_MESSAGE_HEADER:
                {
                    if (bufferTemp[i] == HEADER_EMP)
                    {
                        header = &bufferTemp[i];
                        currMessageState = MESSAGE_HEADER_FOUND;
                    }
                    break;
                }
                case MESSAGE_HEADER_FOUND:
                {
                    if (bufferTemp[i] == HEADER_ENGINE_MONITOR_MESSAGE)
                    {
                        if ((header - &bufferTemp[0]) + 1 == i)
                        {
                            currMessageState = NORMAL_MODE_MESSAGE_ID_FOUND;
                        }
                        else
                        {
                            currMessageState = NO_MESSAGE_HEADER;
                            header = NULL;
                        }
                    }
                    else if (bufferTemp[i] == HEADER_ENTER_TEST_MODE)
                    {
                        if ((header - &bufferTemp[0]) + 1 == i)
                        {
                            currMessageState = IBIT_MODE_MESSAGE_ID_FOUND;
                        }
                        else
                        {
                            currMessageState = NO_MESSAGE_HEADER;
                            header = NULL;
                        }
                    }
                    else if (bufferTemp[i] == HEADER_ENTER_BINGO_SET_MODE)
                    {
                        if ((header - &bufferTemp[0]) + 1 == i)
                        {
                            currMessageState = BINGO_MODE_MESSAGE_ID_FOUND;
                        }
                        else
                        {
                            currMessageState = NO_MESSAGE_HEADER;
                            header = NULL;
                        }
                    }
                    break;
                }
                case NORMAL_MODE_MESSAGE_ID_FOUND:
                {
                    // todo change state to ready after get completed one frame data/
                    uint8_t lengthFromHeader = (i - (header - &bufferTemp[0])) + 1;
                    // printf("lengthfromheader: %d\n", lengthFromHeader);
                    // printf("i: %d\n", i);
                    if ((lengthFromHeader) == sizeOfNormalMode())
                    {
                        // printf("length: %d\n", (header - &bufferTemp[0]));
                        dataParsed = new uint8_t[sizeOfNormalMode()];
                        memcpy(dataParsed, &bufferTemp[header - &bufferTemp[0]], sizeOfNormalMode());
                        currMessageState = MESSAGE_READY_PASSED;
                    }
                    else if (lengthFromHeader > sizeOfNormalMode())
                    {
                        currMessageState = NO_MESSAGE_HEADER;
                        header = NULL;
                    }
                    break;
                }
                case IBIT_MODE_MESSAGE_ID_FOUND:
                {
                    if (bufferTemp[i] == TERMINATOR_EXIT_TEST_MODE)
                    {
                        uint8_t lengthIbitData = i - (header - &bufferTemp[0]);
                        dataParsed = new uint8_t[lengthIbitData];
                        memcpy(dataParsed, &bufferTemp[header - &bufferTemp[0]], lengthIbitData);
                        currMessageState = MESSAGE_READY_PASSED;
                    }
                    break;
                }
                case BINGO_MODE_MESSAGE_ID_FOUND:
                {
                    uint8_t lengthFromHeaderBingo = i - (header - &bufferTemp[0]);
                    if (lengthFromHeaderBingo == sizeOfBingoMode())
                    {
                        dataParsed = new uint8_t[sizeOfBingoMode()];
                        memcpy(dataParsed, &bufferTemp[header - &bufferTemp[0]], sizeOfBingoMode());
                        currMessageState = MESSAGE_READY_PASSED;
                    }
                    else if (lengthFromHeaderBingo > sizeOfBingoMode())
                    {
                        currMessageState = NO_MESSAGE_HEADER;
                        header = NULL;
                    }
                    break;
                    break;
                }
                case MESSAGE_READY_PASSED:
                {
                    // todo process emp parser and set state to initial values/
                    this->processData(dataParsed);
                    // free memory
                    delete[] dataParsed;
                    // restucture buffer temp
                    uint8_t indexHeader = (header - &bufferTemp[0]);
                    uint8_t indexNext = indexHeader + sizeOfNormalMode();
                    memcpy(&bufferTemp[0], &bufferTemp[indexNext], indexNext);

                    // printf("===BUFFER DATA AFTER===\n");
                    // printf("%d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d -%d\n",
                    //        bufferTemp[0], bufferTemp[1], bufferTemp[2], bufferTemp[3], bufferTemp[4], bufferTemp[5],
                    //        bufferTemp[6], bufferTemp[7], bufferTemp[8], bufferTemp[9], bufferTemp[10], bufferTemp[11],
                    //        bufferTemp[12], bufferTemp[13], bufferTemp[14], bufferTemp[15], bufferTemp[16], bufferTemp[17],
                    //        bufferTemp[18], bufferTemp[19], bufferTemp[20]);

                    arraySize -= (indexNext);
                    //printf("arraysize after: %d\n", arraySize);
                    header = NULL;
                    currMessageState = NO_MESSAGE_HEADER;
                    break;
                }
                default:
                    break;
                }
            }
        }
    }

    // If code reaches here, read must of been successful
}

int32_t SerialPort::Available()
{
    if (state_ != State::OPEN)
        THROW_EXCEPT(std::string() + __PRETTY_FUNCTION__ + " called but state != OPEN. Please call Open() first.");
    int32_t ret = 0;
    ioctl(fileDesc_, FIONREAD, &ret);
    return ret;
}

State SerialPort::GetState()
{
    return state_;
}

void SerialPort::processData(uint8_t *data)
{
    if (data[0] == HEADER_EMP && data[1] == HEADER_ENGINE_MONITOR_MESSAGE)
    {
        // normalModeMessage.tl_tm = ((data[3] << 8) + data[2]) * 0.1f;
        // normalModeMessage.ul_um = ((data[5] << 8) + data[4]) * 0.1f;
        // normalModeMessage.vl_vm = (data[7] << 8) + data[6];
        // normalModeMessage.ww = data[8];
        // normalModeMessage.yy = data[9] * 10;
        // normalModeMessage.zz = data[10] * 10;
        // normalModeMessage.ql_qm = ((data[12] << 8) + data[11]) * 10;
        // normalModeMessage.rr = data[13];
        // normalModeMessage.sl_sm = (data[15] << 8) + data[14];
        // normalModeMessage.fcl_fcm = (data[17] << 8) + data[16];
        printf("===PARSED DATA===\n");
        printf("%d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d - %d\n",
               data[0], data[1], data[2], data[3], data[4], data[5],
               data[6], data[7], data[8], data[9], data[10], data[11],
               data[12], data[13], data[14], data[15], data[16], data[17],
               data[18], data[19], data[20]);

        // printf(":: NORMAL MODE MESSAGE ::\n");
        // printf("  TL_TM    : 0x%02X 0x%02X\n", data[3], data[2]);
        // printf("  UL_UM    : 0x%02X 0x%02X\n", data[5], data[4]);
        // printf("  VL_VM    : 0x%02X 0x%02X\n", data[7], data[6]);
        // printf("  WW       : 0x%02X      \n", data[8]);
        // printf("  YY       : 0x%02X      \n", data[9]);
        // printf("  ZZ       : 0x%02X      \n", data[10]);
        // printf("  QL_QM    : 0x%02X 0x%02X\n", data[12], data[11]);
        // printf("  RR       : 0x%02X      \n", data[13]);
        // printf("  SL_SM    : 0x%02X 0x%02X \n", data[15], data[14]);
        // printf("  FCL_FCM  : 0x%02X 0x%02X \n", data[17], data[16]);
    }
}
