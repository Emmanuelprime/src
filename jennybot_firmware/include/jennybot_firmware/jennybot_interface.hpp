#ifndef JENNYBOT_HARDWARE_INTERFACE_HPP
#define JENNYBOT_HARDWARE_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <libserial/SerialPort.h>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <queue>
#include <regex>
#include <chrono>
#include <unordered_map>

namespace jennybot_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Configuration structures
struct SerialConfig {
    std::string port;
    int baud_rate = 115200;
    int data_bits = 8;
    LibSerial::Parity parity = LibSerial::Parity::PARITY_NONE;
    LibSerial::StopBits stop_bits = LibSerial::StopBits::STOP_BITS_1;
    int timeout_ms = 100;
    bool enable_checksum = true;
};

struct JointConfig {
    std::string name;
    double max_velocity = 10.0;
    double max_acceleration = 5.0;
    double encoder_cpr = 13.0;
    double wheel_radius = 0.042;
    bool invert_direction = false;
    double gear_ratio = 16.0;
};

struct RobotConfig {
    std::string robot_type = "differential_drive";
    double wheel_separation = 0.5;
    double wheel_radius = 0.1;
    int control_frequency = 50;
    int watchdog_timeout_ms = 500;
    int max_reconnect_attempts = 3;
};

// Message Protocol Handler
class MessageProtocol {
public:
    static constexpr char MSG_START = '$';
    static constexpr char MSG_END = '\n';
    static constexpr char FIELD_SEPARATOR = ',';
    static constexpr char CHECKSUM_MARKER = '*';
    
    struct ParsedMessage {
        std::unordered_map<std::string, double> values;
        bool valid = false;
        std::string error;
        std::string raw;
        uint64_t timestamp_ms;
    };
    
    static std::string createCommand(const std::vector<std::pair<std::string, double>>& commands) {
        std::stringstream ss;
        ss << MSG_START;
        
        for (size_t i = 0; i < commands.size(); ++i) {
            const auto& cmd = commands[i];
            char sign = cmd.second >= 0 ? '+' : '-';
            ss << cmd.first << sign << std::fixed << std::setprecision(2) 
               << std::setw(6) << std::setfill('0') << std::abs(cmd.second);
            
            if (i < commands.size() - 1) {
                ss << FIELD_SEPARATOR;
            }
        }
        
        // Add checksum
        std::string msg_without_checksum = ss.str();
        uint8_t checksum = calculateChecksum(msg_without_checksum.substr(1));
        ss << CHECKSUM_MARKER << std::hex << std::uppercase 
           << std::setw(2) << std::setfill('0') << static_cast<int>(checksum);
        ss << MSG_END;
        
        return ss.str();
    }
    
    static ParsedMessage parseMessage(const std::string& raw) {
        ParsedMessage result;
        result.raw = raw;
        result.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        
        // Validate basic format
        if (raw.empty() || raw[0] != MSG_START) {
            result.error = "Invalid message start marker";
            return result;
        }
        
        if (raw.back() != MSG_END) {
            result.error = "Invalid message end marker";
            return result;
        }
        
        // Find checksum position
        size_t checksum_pos = raw.find(CHECKSUM_MARKER);
        if (checksum_pos == std::string::npos) {
            result.error = "No checksum found";
            return result;
        }
        
        // Extract checksum
        std::string expected_checksum_str = raw.substr(checksum_pos + 1, 2);
        std::string msg_without_checksum = raw.substr(1, checksum_pos - 1);
        
        // Verify checksum
        uint8_t calculated_checksum = calculateChecksum(msg_without_checksum);
        uint8_t received_checksum = std::stoi(expected_checksum_str, nullptr, 16);
        
        if (calculated_checksum != received_checksum) {
            result.error = "Checksum mismatch. Expected: " + 
                          std::to_string(calculated_checksum) + 
                          ", Received: " + std::to_string(received_checksum);
            return result;
        }
        
        // Parse fields using regex
        std::regex field_pattern(R"(([a-zA-Z])([+\-])([0-9.]+))");
        std::string data_part = msg_without_checksum;
        
        auto fields_begin = std::sregex_iterator(data_part.begin(), data_part.end(), field_pattern);
        auto fields_end = std::sregex_iterator();
        
        for (std::sregex_iterator i = fields_begin; i != fields_end; ++i) {
            std::smatch match = *i;
            std::string motor_id = match[1].str();
            char sign = match[2].str()[0];
            double value = std::stod(match[3].str());
            
            result.values[motor_id] = (sign == '+') ? value : -value;
        }
        
        result.valid = !result.values.empty();
        return result;
    }
    
private:
    static uint8_t calculateChecksum(const std::string& data) {
        uint8_t checksum = 0;
        for (char c : data) {
            checksum ^= static_cast<uint8_t>(c);
        }
        return checksum;
    }
};

// Serial Port Manager
class SerialPortManager {
public:
    explicit SerialPortManager(const SerialConfig& config) 
        : config_(config), is_connected_(false) {
        serial_port_ = std::make_unique<LibSerial::SerialPort>();
    }
    
    ~SerialPortManager() {
        disconnect();
    }
    
    bool connect() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        try {
            if (is_connected_) {
                disconnect();
            }
            
            serial_port_->Open(config_.port);
            serial_port_->SetBaudRate(static_cast<LibSerial::BaudRate>(config_.baud_rate));
            serial_port_->SetCharacterSize(static_cast<LibSerial::CharacterSize>(config_.data_bits));
            serial_port_->SetParity(config_.parity);
            serial_port_->SetStopBits(config_.stop_bits);
            
            // Note: LibSerial timeout is handled via IsDataAvailable(), not SetReadTimeout()
            
            is_connected_ = true;
            last_activity_ = std::chrono::steady_clock::now();
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("SerialPortManager"), 
                               "Failed to connect: " << e.what());
            is_connected_ = false;
            return false;
        }
    }
    
    void disconnect() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (serial_port_ && serial_port_->IsOpen()) {
            try {
                serial_port_->Close();
            } catch (...) {
                // Log but don't throw
            }
        }
        is_connected_ = false;
    }
    
    bool writeMessage(const std::string& message) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!is_connected_ || !serial_port_->IsOpen()) {
            return false;
        }
        
        try {
            serial_port_->Write(message);
            last_activity_ = std::chrono::steady_clock::now();
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("SerialPortManager"), 
                               "Write failed: " << e.what());
            is_connected_ = false;
            return false;
        }
    }
    
    std::vector<std::string> readAvailableMessages() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        std::vector<std::string> messages;
        
        if (!is_connected_ || !serial_port_->IsOpen()) {
            return messages;
        }
        
        try {
            // Read available data into buffer
            while (serial_port_->IsDataAvailable()) {
                char c;
                serial_port_->ReadByte(c);
                rx_buffer_ += c;
                
                // Extract complete messages
                size_t end_pos;
                while ((end_pos = rx_buffer_.find(MessageProtocol::MSG_END)) != std::string::npos) {
                    std::string message = rx_buffer_.substr(0, end_pos + 1);
                    rx_buffer_.erase(0, end_pos + 1);
                    
                    if (message[0] == MessageProtocol::MSG_START) {
                        messages.push_back(message);
                    }
                }
            }
            
            if (!messages.empty()) {
                last_activity_ = std::chrono::steady_clock::now();
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("SerialPortManager"), 
                               "Read failed: " << e.what());
            is_connected_ = false;
        }
        
        return messages;
    }
    
    bool isHealthy() const {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_activity_).count();
        return is_connected_ && elapsed < config_.timeout_ms * 3;
    }
    
    bool isConnected() const {
        return is_connected_ && serial_port_ && serial_port_->IsOpen();
    }
    
private:
    SerialConfig config_;
    std::unique_ptr<LibSerial::SerialPort> serial_port_;
    std::atomic<bool> is_connected_;
    std::mutex mutex_;
    std::string rx_buffer_;
    std::chrono::steady_clock::time_point last_activity_;
};

// Diagnostics Manager
class DiagnosticsManager {
public:
    DiagnosticsManager() {
        stats_.messages_sent = 0;
        stats_.messages_received = 0;
        stats_.checksum_errors = 0;
        stats_.parse_errors = 0;
        stats_.connection_drops = 0;
    }
    
    void initializeDiagnostics(rclcpp::Node* node) {
        diag_publisher_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10);
    }
    
    void publishDiagnostics(bool is_connected, const std::string& status_msg = "") {
        if (!diag_publisher_) return;
        
        diagnostic_msgs::msg::DiagnosticArray diag_array;
        diag_array.header.stamp = rclcpp::Clock().now();
        
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "JennyBot Hardware Interface";
        status.hardware_id = "jennybot_controller";
        
        if (is_connected) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Connected and operational";
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = status_msg.empty() ? "Connection lost" : status_msg;
        }
        
        // Add statistics
        status.values.push_back(makeKeyValue("Messages Sent", stats_.messages_sent));
        status.values.push_back(makeKeyValue("Messages Received", stats_.messages_received));
        status.values.push_back(makeKeyValue("Checksum Errors", stats_.checksum_errors));
        status.values.push_back(makeKeyValue("Parse Errors", stats_.parse_errors));
        status.values.push_back(makeKeyValue("Connection Drops", stats_.connection_drops));
        
        double error_rate = stats_.messages_received > 0 ? 
            static_cast<double>(stats_.checksum_errors + stats_.parse_errors) / 
            stats_.messages_received * 100.0 : 0.0;
        status.values.push_back(makeKeyValue("Error Rate (%)", error_rate));
        
        diag_array.status.push_back(status);
        diag_publisher_->publish(diag_array);
    }
    
    struct Statistics {
        std::atomic<uint64_t> messages_sent;
        std::atomic<uint64_t> messages_received;
        std::atomic<uint64_t> checksum_errors;
        std::atomic<uint64_t> parse_errors;
        std::atomic<uint64_t> connection_drops;
    } stats_;
    
private:
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_publisher_;
    
    diagnostic_msgs::msg::KeyValue makeKeyValue(const std::string& key, double value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = std::to_string(value);
        return kv;
    }
    
    diagnostic_msgs::msg::KeyValue makeKeyValue(const std::string& key, uint64_t value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = std::to_string(value);
        return kv;
    }
};

// Main Hardware Interface Class
class JennyBotInterface : public hardware_interface::SystemInterface
{
public:
    JennyBotInterface();
    virtual ~JennyBotInterface();
    
    // Lifecycle management
    virtual CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
    virtual CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    
    // Hardware interface methods
    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    virtual hardware_interface::return_type read(const rclcpp::Time& time, 
                                                 const rclcpp::Duration& period) override;
    virtual hardware_interface::return_type write(const rclcpp::Time& time, 
                                                  const rclcpp::Duration& period) override;

private:
    // Configuration
    SerialConfig serial_config_;
    RobotConfig robot_config_;
    std::vector<JointConfig> joint_configs_;
    
    // Hardware components
    std::unique_ptr<SerialPortManager> serial_manager_;
    std::unique_ptr<DiagnosticsManager> diagnostics_;
    
    // State and command vectors
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> effort_states_;
    std::vector<double> velocity_commands_;
    
    // Thread safety
    mutable std::mutex data_mutex_;
    std::atomic<bool> emergency_stop_{false};
    
    // Timing and monitoring
    rclcpp::Time last_read_time_;
    rclcpp::Time last_write_time_;
    std::chrono::steady_clock::time_point last_valid_message_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    
    // Statistics
    uint32_t reconnect_attempts_{0};
    uint32_t message_drops_{0};
    
    // Helper methods
    void parseConfiguration(const hardware_interface::HardwareInfo& info);
    bool configureSerialPort();
    bool testCommunication();
    void emergencyStop();
    void watchdogCallback();
    void diagnosticsCallback();
    bool attemptReconnection();
    void applyVelocityLimits();
    hardware_interface::return_type processIncomingMessages();
    bool sendCommands();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
};

} 

#endif