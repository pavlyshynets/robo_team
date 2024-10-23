#include "serial_handler.hpp"

namespace serial_communication
{

SerialHandler::SerialHandler()
: Node("serial_handler"), is_port_open_(false)
{
    // Declare and load parameters
    declare_parameters();
    load_parameters();

    // Configure serial port
    if (!configure_serial_port()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure serial port");
        return;
    }

    // Initialize subscriber
    data_subscriber_ = this->create_subscription<custom_interfaces::msg::SerialData>(
        "serial_write_topic", 10,
        std::bind(&SerialHandler::data_callback, this, std::placeholders::_1));

    // Initialize publisher
    data_publisher_ = this->create_publisher<custom_interfaces::msg::SerialData>(
        "serial_read_topic", 10);

    // Create timer for reading serial data
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
        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(baud_rate_);
        serial_port_.setTimeout(serial::Timeout::simpleTimeout(timeout_ms_));

        serial_port_.open();
        is_port_open_ = serial_port_.isOpen();

        if (is_port_open_) {
            RCLCPP_INFO(this->get_logger(), "Successfully opened serial port");
            return true;
        }
    }
    catch (const serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
    }
    catch (const serial::SerialException& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial exception: %s", e.what());
    }

    return false;
}

void SerialHandler::close_serial_port()
{
    if (is_port_open_) {
        serial_port_.close();
        is_port_open_ = false;
        RCLCPP_INFO(this->get_logger(), "Closed serial port");
    }
}

void SerialHandler::data_callback(const custom_interfaces::msg::SerialData::SharedPtr msg)
{
    if (!is_port_open_) {
        RCLCPP_WARN(this->get_logger(), "Serial port is not open");
        return;
    }

    try {
        // Write data to serial port
        size_t bytes_written = serial_port_.write(msg->data);
        RCLCPP_DEBUG(this->get_logger(), "Written %zu bytes to serial port", bytes_written);
    }
    catch (const serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port: %s", e.what());
    }
}

void SerialHandler::read_serial_data()
{
    if (!is_port_open_) {
        return;
    }

    try {
        // Check if there's data available to read
        if (serial_port_.available()) {
            std::vector<int8_t> buffer(serial_port_.available());
            size_t bytes_read = serial_port_.read(buffer.data(), buffer.size());

            if (bytes_read > 0) {
                // Create and publish message
                auto msg = custom_interfaces::msg::SerialData();
                msg.data = buffer;
                data_publisher_->publish(msg);

                RCLCPP_DEBUG(this->get_logger(), "Read and published %zu bytes", bytes_read);
            }
        }
    }
    catch (const serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read from serial port: %s", e.what());
    }
}

} // namespace serial_communication

// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<serial_communication::SerialHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}