#include <chrono>
#include <memory>
#include <functional>
#include <stdexcept>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "sen0412_ros2/h3lis200dl.hpp"

using namespace std::chrono_literals;

namespace sen0412_ros2 {

class Sen0412Node : public rclcpp::Node
{
public:
    Sen0412Node(): Node("sen0142_node"){

        // rclcpp parameter declarations
        declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
        declare_parameter<std::string>("frame_id", "imu_link");
        declare_parameter<int>("i2c_address", 0x19);
        declare_parameter<double>("range", 200.0);
        declare_parameter<double>("output_data_rate", 1000.0);
        declare_parameter<double>("publish_rate", 200.0);

        // i2c device file
        const auto i2c_device = get_parameter("i2c_device").as_string();
        
        // publisher
        const int i2c_address = get_parameter("i2c_address").as_int();
        const std::string frame_id = get_parameter("frame_id").as_string();
        const double range = get_parameter("range").as_double();
        const double output_data_rate = get_parameter("output_data_rate").as_double();
        const double publish_rate = get_parameter("publish_rate").as_double();

        accelerometer_ = std::make_unique<sen0412_ros2::H3LIS200DL>(i2c_device, static_cast<uint8_t>(i2c_address));

        accelerometer_->initialize(output_data_rate, range);

        accelerometer_publisher_ = create_publisher<sensor_msgs::msg::Imu>("accelerometer", rclcpp::SensorDataQoS());

        const auto period = std::chrono::duration<double>(1.0 / publish_rate);

        accelerometer_timer_ = create_wall_timer(period, std::bind(&Sen0412Node::read, this));
    
    }

private:
    void read(){
        Acceleration acceleration = accelerometer_->read();

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
        accelerometer_publisher_->publish(msg);

    }

    std::unique_ptr<sen0412_ros2::H3LIS200DL> accelerometer_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accelerometer_publisher_;

    rclcpp::TimerBase::SharedPtr accelerometer_timer_;

    std::string frame_id; 
};
}
        
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sen0412_ros2::Sen0412Node>());
    rclcpp::shutdown();

    return 0;
}