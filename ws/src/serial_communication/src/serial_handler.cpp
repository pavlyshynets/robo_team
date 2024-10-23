// serial_handler.cpp
#include "serial_handler.hpp"

namespace serial_communication
{

SerialHandler::SerialHandler()
: Node("serial_handler"), is_port_open_(false)
{
    declare_parameters();
    load_parameters();
    
    if (!configure_serial_port()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure serial port");
        return;
    }
    
    data_subscriber_ = this->create_subscription<custom_interfaces::msg::SerialData>(
        "serial_write_topic", 10,
        std::bind(&SerialHandler::data_callback, this, std::placeholders::_1));
        
    data_publisher_ = this->create_publisher<custom_interfaces::msg::SerialData>(
        "serial_read_topic", 10);
        
    read_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&SerialHandler::read_serial_data, this));
        
    RCLCPP_INFO(this->get_logger(), "Serial handler node initialized successfully");
}

SerialHandler::~SerialHandler()
{
    close_serial_port();
}

void SerialHandler::declare_parameters()
{
    this->declare_parameter("port_name", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("timeout_ms", 1000);
}

void SerialHandler::load_parameters()
{
    port_name_ = this->get_parameter("port_name").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    timeout_ms_ = this->get_parameter("timeout_ms").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Loaded parameters: port=%s, baud_rate=%d",
                port_name_.c_str(), baud_rate_);
}

bool SerialHandler::configure_serial_port()
{
    try {
        serial_port_.Open(port_name_);
        serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200); // Set according to baud_rate_
        serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
        serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        
        is_port_open_ = true;
        RCLCPP_INFO(this->get_logger(), "Successfully opened serial port");
        return true;
    }
    catch (const LibSerial::OpenFailed& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial exception: %s", e.what());
    }
    
    return false;
}

void SerialHandler::close_serial_port()
{
    if (is_port_open_) {
        try {
            serial_port_.Close();
            is_port_open_ = false;
            RCLCPP_INFO(this->get_logger(), "Closed serial port");
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error closing serial port: %s", e.what());
        }
    }
}

bool SerialHandler::validate_message_size(const std::vector<int8_t>& data)
{
    if (data.size() != SERIAL_MSG_SIZE) {
        RCLCPP_WARN(this->get_logger(), 
            "Invalid message size. Expected %zu bytes, got %zu bytes", 
            SERIAL_MSG_SIZE, data.size());
        return false;
    }
    return true;
}

void SerialHandler::data_callback(const custom_interfaces::msg::SerialData::SharedPtr msg)
{
    if (!is_port_open_) {
        RCLCPP_WARN(this->get_logger(), "Serial port is not open");
        return;
    }
    
    if (!validate_message_size(msg->data)) {
        return;
    }
    
    try {
        std::string data_str(msg->data.begin(), msg->data.end());
        serial_port_.Write(data_str);
        RCLCPP_DEBUG(this->get_logger(), "Written %zu bytes to serial port", SERIAL_MSG_SIZE);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", e.what());
    }
}

void SerialHandler::read_serial_data()
{
    if (!is_port_open_) {
        return;
    }
    
    try {
        if (serial_port_.IsDataAvailable()) {
            std::string received_data;
            serial_port_.Read(received_data, SERIAL_MSG_SIZE, timeout_ms_);
            
            if (received_data.length() == SERIAL_MSG_SIZE) {
                auto msg = custom_interfaces::msg::SerialData();
                msg.data.assign(received_data.begin(), received_data.end());
                data_publisher_->publish(msg);
                
                RCLCPP_DEBUG(this->get_logger(), "Read and published %zu bytes", SERIAL_MSG_SIZE);
            }
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read from serial port: %s", e.what());
    }
}

} // namespace serial_communication

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<serial_communication::SerialHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
