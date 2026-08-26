#include <chrono>
#include <memory>
#include <functional>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "sen0412_ros2/h3lis200dl.hpp"

using namespace std::chrono_literals;

class Sen0412Node : public rclcpp::Node
{
public:
    Sen0412Node(): Node("sen0142_node"){

        // rclcpp parameter declarations
        declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
        declare_parameter<std::string>("frame_id", "imu_link");
        declare_parameter<int>("i2c_address", 0x19);
        declare_parameter<int>("range", 100);
        declare_parameter<double>("output_data_rate", 1000);
        declare_parameter<double>("publish_rate", 200);

        // i2c device file
        const auto i2c_device = get_parameter("i2c_device").as_string();
        
        // publisher
        const int i2c_address = get_parameter("address").as_int();
        frame_id = get_paramter("frame_id").as_string();
        const int range = get_parameter("range").as_int();
        const double output_data_rate = get_parameter("output_data_rate").as_double();
        const double publish_rate = get_parameter("publish_rate").as_double();
        
        accelerometer_ = std::make_unique<sen0140_ros2::H3LIS200DL>(i2c_device, static_cast<uint8_t>(i2c_address));

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

        Range range_e;
        switch (static_cast<Range>(range)) {
            case Range::e100g:
            case Range::e200g:
                range_e = static_cast<Range>(range);
                break;
            default:
                throw std::runtime_error("Invalid range for H3LIS200DL");
        }

        accelerometer_->initialize(output_data_rate_e, range_e);

        accelerometer_publisher_ = create_publisher<sensor_msgs::msg::Imu>("accelerometer", rclcpp::SensorDataQoS());

        const auto period = std::chrono::duration<double>(1.0 / mag_publish_rate);

        accelerometer_timer_ = create_wall_timer(period, std::bind(&Sen0412Node::read, this))

private:
    void read(){
        Acceleration acceleration = accelerometer->read()

        // Create ROS2 message
        sensor_msgs::msg::Imu msg;

        msg.header.stamp = now();
        msg.header.frame_id = frame_id;
        
        msg.linear_acceleration.x = acceleration.x;
        msg.linear_acceleration.y = acceleration.y;
        msg.linear_acceleration.z = acceleration.z;

        // docs: First element of orientation_covariance should be -1 if orientation is unknown
        msg.orientation_covariance[0] = -1.0;

        // publish message
        accelerometer_publisher_.publish(msg);

    }

    std::unique_ptr<sen0412_ros2::H3LIS200DL> accelerometer_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accelerometer_publisher_;

    rclcpp::TimerBase::SharedPtr accelerometer_timer_;

    std::string frame_id;
};
        
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sen0412Node>());
    rclcpp::shutdown();

    return 0;
}

};