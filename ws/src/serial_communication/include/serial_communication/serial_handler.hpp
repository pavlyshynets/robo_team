// serial_handler.hpp
#ifndef SERIAL_HANDLER_HPP
#define SERIAL_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialPort.h>
#include "custom_interfaces/msg/serial_data.hpp"
#include <vector>
#include <string>

namespace serial_communication
{

class SerialHandler : public rclcpp::Node
{
public:
    SerialHandler();
    ~SerialHandler();

private:
    static const size_t SERIAL_MSG_SIZE = 34;  // Fixed size of 34 bytes
    
    // Serial port configuration parameters
    std::string port_name_;
    uint32_t baud_rate_;
    uint32_t timeout_ms_;
    LibSerial::SerialPort serial_port_;
    bool is_port_open_;
    
    // ROS2 publishers and subscribers
    rclcpp::Subscription<custom_interfaces::msg::SerialData>::SharedPtr data_subscriber_;
    rclcpp::Publisher<custom_interfaces::msg::SerialData>::SharedPtr data_publisher_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    
    // Parameter handling
    void declare_parameters();
    void load_parameters();
    
    // Callback functions
    void data_callback(const custom_interfaces::msg::SerialData::SharedPtr msg);
    void read_serial_data();
    
    // Serial port handling
    bool configure_serial_port();
    void close_serial_port();
    
    // Utility functions
    bool validate_message_size(const std::vector<int8_t>& data);
};

} // namespace serial_communication

#endif // SERIAL_HANDLER_HPP
