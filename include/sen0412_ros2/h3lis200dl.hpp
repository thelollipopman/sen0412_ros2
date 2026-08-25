#ifndef H3LIS200DL_HPP
#define H3LIS200DL_HPP

#include <cstddef>
#include <cstdint>
#include <string>


namespace sen0412_ros2{

// Acceleration struct
struct Acceleration
{
    double x;
    double y;
    double z;
};

// Output data rate enum
enum OutputDataRate: uint8_t{
    e0HZ = 0,/*Measurement off*/
    e0_5HZ = 0x40,/*0.5 hz*/
    e1HZ  = 0x60,
    e2HZ  = 0x80,
    e5HZ  = 0xA0,
    e10HZ = 0xC0,
    e50HZ   = 0x20,
    e100HZ  = 0x28,
    e400HZ  = 0x30,
    e1000HZ = 0x38,
};

enum Range{
    e100g = 100,/**< ±100g>*/
    e200g = 200,/**< ±200g>*/       
};


class H3LIS200DL {

public: 

    // Constructor
    explicit H3LIS200DL(
        const std::string& i2c_device,
        uint8_t address=0x19
    );

    // Destructor
    ~H3LIS200DL();

    // Set sensor settings
    void initialize(
        OutputDataRate output_data_rate,
        Range range
    );

    Acceleration read();

private: 
    // Registers 
    static constexpr uint8_t REG_WHO_AM_I =        0x0F;     ///<Chip id
    static constexpr uint8_t REG_CTRL_REG1 =       0x20;     ///<Control register 1
    static constexpr uint8_t REG_CTRL_REG4 =       0x23;     ///<Control register 4
    static constexpr uint8_t REG_CTRL_REG2 =       0x21;     ///<Control register 2
    static constexpr uint8_t REG_CTRL_REG3 =       0x22;     ///<Control register 3
    static constexpr uint8_t REG_CTRL_REG5 =       0x24;     ///<Control register 5
    static constexpr uint8_t REG_HP_FILTER_RESET = 0x25;     ///<Read register to zero content of internal high pass filter
    static constexpr uint8_t REG_REFERENCE =       0x26;
    static constexpr uint8_t REG_STATUS_REG =      0x27;     ///<Status register
    static constexpr uint8_t REG_OUT_X =           0x29;     ///<X-axis acceleration register
    static constexpr uint8_t REG_OUT_Y =           0x2B;     ///<Y-axis acceleration register
    static constexpr uint8_t REG_OUT_Z =           0x2D;     ///<Z-axis acceleration register
    static constexpr uint8_t REG_INT1_THS =        0x32;     ///<Interrupt source 1 threshold
    static constexpr uint8_t REG_INT2_THS =        0x36;     ///<Interrupt source 2 threshold
    static constexpr uint8_t REG_INT1_CFG =        0x30;     ///<Interrupt source 1 configuration register
    static constexpr uint8_t REG_INT2_CFG =        0x34;     ///<Interrupt source 2 configuration register
    static constexpr uint8_t REG_INT1_SRC =        0x31;     ///<Interrupt source 1 status register
    static constexpr uint8_t REG_INT2_SRC =        0x35;     ///<Interrupt source 2 status register

    // Member variables
    int fd_;
    int range_;

    // Helper functions for I2C read write
    void write_register(
        uint8_t reg,
        uint8_t value);

    uint8_t read_register(
        uint8_t reg);

    void read_registers(
        uint8_t start_reg,
        uint8_t * buffer,
        std::size_t length);

    // Setter functions
    void set_output_data_rate(OutputDataRate rate);

    void set_range(Range range);
};
}
#endif