#include "sen0412_ros2/h3lis200dl.hpp"

#include <stdexcept>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <cmath>
#include <iostream>

namespace sen0412_ros2 {

// Constructor
H3LIS200DL::H3LIS200DL(
    const std::string& i2c_device, 
    uint8_t i2c_address): 
    fd_{-1}, range_{200.0}
{
    // Open I2C device file with read + write permissions, save file descriptor
    fd_ = ::open(i2c_device.c_str(), O_RDWR);

    if (fd_ < 0) {
        throw std::runtime_error(
            "Failed to open I2C device");
    }

    // Configure file to write to I2C address
    if (ioctl(fd_, I2C_SLAVE, i2c_address) < 0) {
        ::close(fd_);
        fd_ = -1;

        throw std::runtime_error(
            "Failed to select H3LIS200DL I2C address");
    }
}


// Destructor
H3LIS200DL::~H3LIS200DL(){
    if (fd_ >= 0){
        ::close(fd_);
    }
};

void H3LIS200DL::initialize(
    double output_data_rate,
    double range
){
    // Check that H3LIS200DL chip id (who_am_i) matches with datasheet
    uint8_t who_am_i = read_register(REG_WHO_AM_I);

    if (who_am_i != static_cast<uint8_t>(0x32)){
        throw std::runtime_error("Unexpected device ID, should be 0x32");
    }

    // User input validation
    OutputDataRateMask output_data_rate_mask;
    if (is_close(output_data_rate, 0.5)) {
        output_data_rate_mask = OutputDataRateMask::e0_5HZ;
    } else if (is_close(output_data_rate, 1.0)) {
        output_data_rate_mask = OutputDataRateMask::e1HZ;
    } else if (is_close(output_data_rate, 2.0)) {
        output_data_rate_mask = OutputDataRateMask::e2HZ;
    } else if (is_close(output_data_rate, 5.0)) {
        output_data_rate_mask = OutputDataRateMask::e5HZ;
    } else if (is_close(output_data_rate, 10.0)) {
        output_data_rate_mask = OutputDataRateMask::e10HZ;
    } else if (is_close(output_data_rate, 50.0)) {
        output_data_rate_mask = OutputDataRateMask::e50HZ;
    } else if (is_close(output_data_rate, 100.0)) {
        output_data_rate_mask = OutputDataRateMask::e100HZ;
    } else if (is_close(output_data_rate, 400.0)) {
        output_data_rate_mask = OutputDataRateMask::e400HZ;
    } else if (is_close(output_data_rate, 1000.0)) {
        output_data_rate_mask = OutputDataRateMask::e1000HZ;
    } else {
        throw std::runtime_error("Invalid output data rate for H3LIS200DL: " + std::to_string(output_data_rate));
    }
    output_data_rate_ = output_data_rate;

    RangeMask range_mask;
    if (is_close(range, 100.0)) {
        range_mask = RangeMask::e100g;
    } else if (is_close(range, 200.0)) {
        range_mask = RangeMask::e200g;
    } else {
        throw std::runtime_error("Invalid range for H3LIS200DL: " + std::to_string(range));
    }
    range_ = range;


    // Set sensor parameters
    set_output_data_rate(output_data_rate_mask);
    set_range(range_mask);
}

void H3LIS200DL::read_registers(
    uint8_t start_reg,
    uint8_t * buffer,
    std::size_t length)
{
    // Master tell slave the starting register address to read from, throw error if cannot
    if (::write(fd_, &start_reg, 1) != 1)
    {
        throw std::runtime_error(
            "Failed to select H3LIS200DL register");
    }

    // Read slave registers into buffer, throw error if read bytes not equal to requested length
    if (::read(fd_, buffer, length) != static_cast<ssize_t>(length))
    {
        throw std::runtime_error(
            "Failed to read H3LIS200DL registers");
    }
}

uint8_t H3LIS200DL::read_register(uint8_t reg)
{
    uint8_t value;

    // Read 1 register into value
    read_registers(reg, &value, 1);

    std::cout << value << std::endl;

    return value;
}

void H3LIS200DL::write_register(uint8_t reg, uint8_t value)
{
    const uint8_t buffer[2] = {
        reg,
        value
    };

    if (::write(
            fd_,
            buffer,
            2) != 2)
    {
        throw std::runtime_error(
            "Failed to write H3LIS200DL register");
    }
}

void H3LIS200DL::set_output_data_rate(OutputDataRateMask rate_mask){
    uint8_t value = read_register(REG_CTRL_REG1);
    
    // output data rate is set by first 5 bits: . Leave remaining bits unchanged.
    value = value & 0b00000111;
    value = value | static_cast<uint8_t>(rate_mask);
    write_register(REG_CTRL_REG1, value);
}

void H3LIS200DL::set_range(RangeMask range_mask){
    uint8_t value = read_register(REG_CTRL_REG4);
    value = value | static_cast<uint8_t>(range_mask);
    write_register(REG_CTRL_REG4, value);
}

Acceleration H3LIS200DL::read(){
    double lsb_to_g = range_ / 128;
    double g_to_ms2 = 9.80665;
    double x_accel = static_cast<int8_t>(read_register(REG_OUT_X)) * lsb_to_g * g_to_ms2;
    double y_accel = static_cast<int8_t>(read_register(REG_OUT_Y)) * lsb_to_g * g_to_ms2;
    double z_accel = static_cast<int8_t>(read_register(REG_OUT_Z)) * lsb_to_g * g_to_ms2;
    std::cout << x_accel << std::endl;
    return {
        x_accel,
        y_accel,
        z_accel
    };
}

bool is_close(double a, double b) {
    return std::abs(a - b) < std::numeric_limits<double>::epsilon();
}




/*

int32_t H3LIS200DL::platform_read(
    void *handle, 
    uint8_t reg, 
    uint8_t *bufp, 
    uint16_t len
){
    // Cast void pointer to H3LIS200DL object type pointer so we can dereference it
    H3LIS200DL* h3lis200dl_ptr = static_cast<H3LIS200DL*>(handle);

    // Write I2C register address to read from; return error status -1 if not successful
    ssize_t bytes_write = ::write(h3lis200dl_ptr -> fd_, &reg, 1);
    if ( bytes_write != 1){
        return -1;
    };

    // Read the register; return error status -1 if not successful
    ssize_t bytes_read = ::read(h3lis200dl_ptr -> fd_, bufp, len);
    if (bytes_read != static_cast<ssize_t>(len)){
        return -1;
    };

    // Successful read
    return 0;
};

int32_t H3LIS200DL::platform_write(
    void *handle, 
    uint8_t reg, 
    const uint8_t *bufp, 
    uint16_t len
){
    // Cast void pointer to H3LIS200DL object type pointer so we can dereference it
    H3LIS200DL* h3lis200dl_ptr = static_cast<H3LIS200DL*>(handle);

    // Initialise buffer to copy bufp data into; reserve 1 extra byte for register address
    std::vector<uint8_t> buffer(len + 1);
    buffer[0] = reg;

    // 

    // Copy rest of I2C message data to buffer
    for (uint16_t i=0; i < len; i++) {
        buffer[i + 1] = bufp[i];
    };

    // Write buffer data to I2C register
    ssize_t bytes_write = ::write(
        h3lis200dl_ptr -> fd_,
        buffer.data(),
        buffer.size()
    );

    // Check if number of bytes written tally with buffer size, else return error status -1
    if (bytes_write != static_cast<ssize_t>(buffer.size())) {
        return -1;
    } 
    return 0;
};
*/

/*
// User input checking output data rate
    OutputDataRate output_data_rate_e; 
    switch (static_cast<OutputDataRate>(output_data_rate)) {
        case OutputDataRate::e0HZ:
        case OutputDataRate::e0_5HZ:
        case OutputDataRate::e1HZ:
        case OutputDataRate::e2HZ:
        case OutputDataRate::e5HZ:
        case OutputDataRate::e10HZ:
        case OutputDataRate::e50HZ:
        case OutputDataRate::e100HZ:
        case OutputDataRate::e400HZ:
        case OutputDataRate::e1000HZ:
            output_data_rate_e = static_cast<OutputDataRate>(output_data_rate);
            break;
        default:
            throw std::runtime_error("Invalid output data rate for H3LIS200DL");
    }

    // Set output data rate safely
    set_output_data_rate(output_data_rate_e);

    // User input checking output data rate
    Range range_e;
    switch (static_cast<Range>(range)) {
        case Range::e100g:
        case Range::e200g:
            range_e = static_cast<Range>(range);
            break;
        default:
            throw std::runtime_error("Invalid range for H3LIS200DL");
    }

    // Set output data rate safely
    set_range(range_e);
*/


// close namespace
}