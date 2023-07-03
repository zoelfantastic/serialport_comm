#ifndef SERIAL_EXCEPTION_H
#define SERIAL_EXCEPTION_H

#include <string>
#include <stdexcept>

class Exception : public std::runtime_error
{
private:
    std::string msg_;

public:
    Exception(const char *file, int line, const std::string &arg) : std::runtime_error(arg)
    {
        msg_ = std::string(file) + ":" + std::to_string(line) + ": " + arg;
    }
    ~Exception() throw() {};
    const char *what() const throw() override
    {
        return msg_.c_str();
    }
};


#define THROW_EXCEPT(arg) throw Exception(__FILE__, __LINE__, arg);
#endif