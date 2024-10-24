#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "serial_communication/msg/int8_array.hpp"  // Custom message for serial communication
#include <cmath>

// CRC-16 Modbus function
uint16_t crc16_modbus(const std::vector<int8_t>& data)
{
    uint16_t crc = 0xFFFF;
    for (auto byte : data) {
        crc ^= static_cast<uint16_t>(byte) & 0xFF;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Convert Twist (velocity) to frequencies for the motors
std::vector<int8_t> toFreqs(float v_x, float v_y, float omega)
{
    const int MAX_FREQ = 10000;
    const float r = 0.1;  // Wheel radius
    const float l_x = 0.1575;  // Distance from the center of the robot to the wheels along the x-axis
    const float l_y = 0.215;   // Distance from the center of the robot to the wheels along the y-axis

    std::vector<int8_t> freqs(4);

    // Calculate the frequencies for each wheel (simplified kinematics)
    freqs[0] = static_cast<int8_t>(MAX_FREQ * round((1/r) * (v_x - v_y - (l_x + l_y) * omega) / (2 * M_PI)));
    freqs[1] = static_cast<int8_t>(MAX_FREQ * round((1/r) * (v_x + v_y + (l_x + l_y) * omega) / (2 * M_PI)));
    freqs[2] = static_cast<int8_t>(MAX_FREQ * round((1/r) * (v_x + v_y - (l_x + l_y) * omega) / (2 * M_PI)));
    freqs[3] = static_cast<int8_t>(MAX_FREQ * round((1/r) * (v_x - v_y + (l_x + l_y) * omega) / (2 * M_PI)));

    return freqs;
}

class TeleopToSerialNode : public rclcpp::Node
{
public:
    TeleopToSerialNode() : Node("teleop_to_serial_node")
    {
        // Subscription to Twist message (from teleop-twist-joy)
        twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&TeleopToSerialNode::twist_callback, this, std::placeholders::_1));

        // Publisher for Int8Array message
        serial_publisher_ = this->create_publisher<serial_communication::msg::Int8Array>("serial_write", 10);
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const int8_t DEVICE_ID = 0xA8;
        const int8_t MESSAGE_LENGTH = 20;
        const int8_t MESSAGE_ID = 1;

        // Convert Twist data to motor frequencies
        auto freqs = toFreqs(msg->linear.x, msg->linear.y, msg->angular.z);

        // Prepare the serial data message
        std::vector<int8_t> serial_data = {
            DEVICE_ID, MESSAGE_LENGTH, MESSAGE_ID,
            static_cast<int8_t>((freqs[0] >> 8) & 0xFF), static_cast<int8_t>(freqs[0] & 0xFF),
            static_cast<int8_t>((freqs[1] >> 8) & 0xFF), static_cast<int8_t>(freqs[1] & 0xFF),
            static_cast<int8_t>((freqs[2] >> 8) & 0xFF), static_cast<int8_t>(freqs[2] & 0xFF),
            static_cast<int8_t>((freqs[3] >> 8) & 0xFF), static_cast<int8_t>(freqs[3] & 0xFF),
            0, 0, 0  // Placeholder bytes for future use
        };

        // Calculate and append CRC to the message
        uint16_t crc = crc16_modbus(serial_data);
        serial_data.push_back(static_cast<int8_t>(crc & 0xFF));        // CRC low byte
        serial_data.push_back(static_cast<int8_t>((crc >> 8) & 0xFF)); // CRC high byte

        // Create the Int8Array message and publish it
        auto serial_msg = serial_communication::msg::Int8Array();
        serial_msg.data = serial_data;
        serial_publisher_->publish(serial_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    rclcpp::Publisher<serial_communication::msg::Int8Array>::SharedPtr serial_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopToSerialNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

