#include <jennybot_firmware/jennybot_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <thread>
#include <future>

// Helper function for safe parameter retrieval
static std::string getParam(const std::unordered_map<std::string, std::string>& params,
                            const std::string& key, const std::string& default_val) {
    auto it = params.find(key);
    return (it != params.end()) ? it->second : default_val;
}

namespace jennybot_firmware
{

JennyBotInterface::JennyBotInterface() 
    : hardware_interface::SystemInterface()
    , logger_(rclcpp::get_logger("JennyBotInterface"))
{
    diagnostics_ = std::make_unique<DiagnosticsManager>();
}

JennyBotInterface::~JennyBotInterface()
{
    if (watchdog_timer_) {
        watchdog_timer_->cancel();
    }
    if (diagnostics_timer_) {
        diagnostics_timer_->cancel();
    }
    
    if (serial_manager_) {
        serial_manager_->disconnect();
    }
}

CallbackReturn JennyBotInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
    RCLCPP_INFO(logger_, "Initializing JennyBotInterface...");
    
    // Call parent initialization
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS) {
        RCLCPP_ERROR(logger_, "Parent initialization failed");
        return result;
    }
    
    // Parse configuration
    parseConfiguration(info_);
    
    // Create node for timers and diagnostics
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    node_ = std::make_shared<rclcpp::Node>("jennybot_hardware_interface", options);
    
    // Initialize diagnostics
    diagnostics_->initializeDiagnostics(node_.get());
    
    // Reserve space for joints
    size_t num_joints = info_.joints.size();
    position_states_.resize(num_joints, 0.0);
    velocity_states_.resize(num_joints, 0.0);
    effort_states_.resize(num_joints, 0.0);
    velocity_commands_.resize(num_joints, 0.0);
    
    // Create serial manager
    serial_manager_ = std::make_unique<SerialPortManager>(serial_config_);
    
    RCLCPP_INFO(logger_, "JennyBotInterface initialized successfully with %zu joints", num_joints);
    return CallbackReturn::SUCCESS;
}

CallbackReturn JennyBotInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Configuring JennyBotInterface...");
    
    // Test serial port configuration
    if (!configureSerialPort()) {
        RCLCPP_ERROR(logger_, "Failed to configure serial port");
        return CallbackReturn::FAILURE;
    }
    
    RCLCPP_INFO(logger_, "JennyBotInterface configured successfully");
    return CallbackReturn::SUCCESS;
}

CallbackReturn JennyBotInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Activating JennyBotInterface...");
    
    // Reset states
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::fill(position_states_.begin(), position_states_.end(), 0.0);
        std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);
        std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
        emergency_stop_ = false;
    }
    RCLCPP_INFO(logger_, "Step 1: States reset");
    
    // Connect to hardware
    RCLCPP_INFO(logger_, "Step 2: Verifying serial port connection...");
    
    // Serial port was already connected in configure(), just verify it's still connected
    if (!serial_manager_->isConnected()) {
        RCLCPP_ERROR(logger_, "Serial port is not connected");
        return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(logger_, "Step 3: Serial port connected");
    
    // Test communication
    RCLCPP_INFO(logger_, "Step 4: Testing communication...");
    if (!testCommunication()) {
        RCLCPP_WARN(logger_, "Communication test failed, but continuing anyway");
    }
    RCLCPP_INFO(logger_, "Step 5: Communication test complete");
    
    // Setup watchdog timer
    RCLCPP_INFO(logger_, "Step 6: Creating watchdog timer...");
    watchdog_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(robot_config_.watchdog_timeout_ms),
        std::bind(&JennyBotInterface::watchdogCallback, this));
    RCLCPP_INFO(logger_, "Step 7: Watchdog timer created");
    
    // Setup diagnostics timer
    RCLCPP_INFO(logger_, "Step 8: Creating diagnostics timer...");
    diagnostics_timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&JennyBotInterface::diagnosticsCallback, this));
    RCLCPP_INFO(logger_, "Step 9: Diagnostics timer created");
    
    last_read_time_ = node_->now();
    last_write_time_ = node_->now();
    last_valid_message_ = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(logger_, "JennyBotInterface activated successfully");
    return CallbackReturn::SUCCESS;
}

CallbackReturn JennyBotInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Deactivating JennyBotInterface...");
    
    // Cancel timers
    if (watchdog_timer_) {
        watchdog_timer_->cancel();
    }
    if (diagnostics_timer_) {
        diagnostics_timer_->cancel();
    }
    
    // Send stop command
    emergencyStop();
    
    // Wait for stop command to be sent
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Disconnect
    serial_manager_->disconnect();
    
    RCLCPP_INFO(logger_, "JennyBotInterface deactivated successfully");
    return CallbackReturn::SUCCESS;
}

CallbackReturn JennyBotInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Cleaning up JennyBotInterface...");
    
    serial_manager_.reset();
    
    RCLCPP_INFO(logger_, "JennyBotInterface cleaned up successfully");
    return CallbackReturn::SUCCESS;
}

CallbackReturn JennyBotInterface::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(logger_, "Shutting down JennyBotInterface...");
    
    if (serial_manager_) {
        serial_manager_->disconnect();
    }
    
    RCLCPP_INFO(logger_, "JennyBotInterface shut down successfully");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JennyBotInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, 
            hardware_interface::HW_IF_POSITION, 
            &position_states_[i]));
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY, 
            &velocity_states_[i]));
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, 
            hardware_interface::HW_IF_EFFORT, 
            &effort_states_[i]));
    }
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JennyBotInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY, 
            &velocity_commands_[i]));
    }
    
    return command_interfaces;
}

hardware_interface::return_type JennyBotInterface::read(const rclcpp::Time& time, 
                                                        const rclcpp::Duration& period)
{
    (void)time; // Suppress unused parameter warning
    
    // Check if we need to reconnect
    if (!serial_manager_->isConnected()) {
        if (!attemptReconnection()) {
            return hardware_interface::return_type::ERROR;
        }
    }
    
    // Process incoming messages
    auto result = processIncomingMessages();
    if (result != hardware_interface::return_type::OK) {
        return result;
    }
    
    // Update position estimates based on velocity
    double dt = period.seconds();
    if (dt > 0.0 && dt < 1.0) { // Sanity check on dt
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < position_states_.size(); i++) {
            position_states_[i] += velocity_states_[i] * dt;
        }
    }
    
    last_read_time_ = node_->now();
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type JennyBotInterface::write(const rclcpp::Time& time, 
                                                         const rclcpp::Duration& period)
{
    (void)time; // Suppress unused parameter warning
    (void)period;
    
    // Debug: Log raw commands from controller
    static int write_log_counter = 0;
    if (write_log_counter++ % 100 == 0) {
        std::stringstream ss;
        ss << "write() called - Raw velocity_commands_: [";
        for (size_t i = 0; i < velocity_commands_.size(); i++) {
            ss << velocity_commands_[i];
            if (i < velocity_commands_.size() - 1) ss << ", ";
        }
        ss << "] rad/s";
        RCLCPP_INFO(logger_, "%s", ss.str().c_str());
    }
    
    // Emergency stop check
    if (emergency_stop_) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
    }
    
    // Apply limits
    applyVelocityLimits();
    
    // Send commands
    if (!sendCommands()) {
        return hardware_interface::return_type::ERROR;
    }
    
    last_write_time_ = node_->now();
    
    return hardware_interface::return_type::OK;
}

void JennyBotInterface::parseConfiguration(const hardware_interface::HardwareInfo& info)
{
    // Parse serial configuration
    auto& hw_params = info.hardware_parameters;
    serial_config_.port = hw_params.at("port");
    serial_config_.baud_rate = std::stoi(getParam(hw_params, "baud_rate", "115200"));
    serial_config_.timeout_ms = std::stoi(getParam(hw_params, "timeout_ms", "100"));
    serial_config_.enable_checksum = getParam(hw_params, "enable_checksum", "true") == "true";
    
    // Parse robot configuration
    robot_config_.robot_type = getParam(hw_params, "robot_type", "differential_drive");
    robot_config_.wheel_separation = std::stod(getParam(hw_params, "wheel_separation", "0.315"));
    robot_config_.wheel_radius = std::stod(getParam(hw_params, "wheel_radius", "0.042"));
    robot_config_.control_frequency = std::stoi(getParam(hw_params, "control_frequency", "50"));
    robot_config_.watchdog_timeout_ms = std::stoi(getParam(hw_params, "watchdog_timeout_ms", "500"));
    robot_config_.max_reconnect_attempts = std::stoi(getParam(hw_params, "max_reconnect_attempts", "3"));
    
    // Parse joint configurations
    joint_configs_.resize(info.joints.size());
    for (size_t i = 0; i < info.joints.size(); i++) {
        const auto& joint = info.joints[i];
        joint_configs_[i].name = joint.name;
        joint_configs_[i].max_velocity = std::stod(getParam(joint.parameters, "max_velocity", "10.0"));
        joint_configs_[i].max_acceleration = std::stod(getParam(joint.parameters, "max_acceleration", "5.0"));
        joint_configs_[i].encoder_cpr = std::stod(getParam(joint.parameters, "encoder_cpr", "13.0"));
        joint_configs_[i].wheel_radius = std::stod(getParam(joint.parameters, "wheel_radius", "0.042"));
        joint_configs_[i].invert_direction = getParam(joint.parameters, "invert_direction", "false") == "true";
        joint_configs_[i].gear_ratio = std::stod(getParam(joint.parameters, "gear_ratio", "16.0"));
    }
}

bool JennyBotInterface::configureSerialPort()
{
    try {
        // Test if we can open the port
        return serial_manager_->connect();
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to configure serial port: " << e.what());
        return false;
    }
}

bool JennyBotInterface::testCommunication()
{
    RCLCPP_INFO(logger_, "Testing communication with hardware...");
    
    // Send a test message (zero velocity command)
    std::vector<std::pair<std::string, double>> test_commands;
    for (size_t i = 0; i < joint_configs_.size(); i++) {
        test_commands.push_back({std::string(1, 'a' + i), 0.0});
    }
    
    std::string test_message = MessageProtocol::createCommand(test_commands);
    if (!serial_manager_->writeMessage(test_message)) {
        RCLCPP_ERROR(logger_, "Failed to send test message");
        return false;
    }
    
    // Wait for response
    auto start_time = std::chrono::steady_clock::now();
    bool response_received = false;
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time).count() < 1000) {
        
        auto messages = serial_manager_->readAvailableMessages();
        if (!messages.empty()) {
            response_received = true;
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    if (response_received) {
        RCLCPP_INFO(logger_, "Communication test successful");
        return true;
    } else {
        RCLCPP_WARN(logger_, "No response received, but continuing anyway");
        return true; // Continue even without response
    }
}

void JennyBotInterface::emergencyStop()
{
    RCLCPP_WARN(logger_, "EMERGENCY STOP ACTIVATED");
    
    emergency_stop_ = true;
    
    // Send zero velocity command multiple times to ensure it's received
    std::vector<std::pair<std::string, double>> stop_commands;
    for (size_t i = 0; i < joint_configs_.size(); i++) {
        stop_commands.push_back({std::string(1, 'a' + i), 0.0});
    }
    
    std::string stop_message = MessageProtocol::createCommand(stop_commands);
    
    for (int i = 0; i < 5; i++) {
        if (serial_manager_ && serial_manager_->isConnected()) {
            serial_manager_->writeMessage(stop_message);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void JennyBotInterface::watchdogCallback()
{
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_valid_message_).count();
    
    if (elapsed > robot_config_.watchdog_timeout_ms) {
        RCLCPP_ERROR_STREAM(logger_, "Watchdog timeout! No valid messages for " 
                           << elapsed << "ms");
        
        emergencyStop();
        diagnostics_->stats_.connection_drops++;
        
        // Attempt reconnection
        if (!serial_manager_->isHealthy()) {
            attemptReconnection();
        }
    }
}

void JennyBotInterface::diagnosticsCallback()
{
    diagnostics_->publishDiagnostics(
        serial_manager_ && serial_manager_->isConnected(),
        emergency_stop_ ? "Emergency stop active" : "");
}

bool JennyBotInterface::attemptReconnection()
{
    if (reconnect_attempts_ >= static_cast<uint32_t>(robot_config_.max_reconnect_attempts)) {
        RCLCPP_ERROR(logger_, "Max reconnection attempts reached");
        emergency_stop_ = true;
        return false;
    }
    
    RCLCPP_INFO(logger_, "Attempting reconnection (%d/%d)", 
                reconnect_attempts_ + 1, robot_config_.max_reconnect_attempts);
    
    reconnect_attempts_++;
    
    // Disconnect first
    serial_manager_->disconnect();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Try to reconnect
    if (serial_manager_->connect() && testCommunication()) {
        RCLCPP_INFO(logger_, "Reconnection successful");
        reconnect_attempts_ = 0;
        last_valid_message_ = std::chrono::steady_clock::now();
        return true;
    }
    
    return false;
}

void JennyBotInterface::applyVelocityLimits()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    for (size_t i = 0; i < velocity_commands_.size() && i < joint_configs_.size(); i++) {
        // Apply inversion first if configured
        double cmd = velocity_commands_[i];
        if (joint_configs_[i].invert_direction) {
            cmd = -cmd;
        }
        
        // Apply max velocity limit (AFTER inversion, BEFORE sending to firmware)
        double max_vel = joint_configs_[i].max_velocity;
        velocity_commands_[i] = std::clamp(cmd, -max_vel, max_vel);
        
        // Note: Gear ratio is handled by the ESP32 firmware, not here
    }
}

hardware_interface::return_type JennyBotInterface::processIncomingMessages()
{
    auto messages = serial_manager_->readAvailableMessages();
    
    if (messages.empty()) {
        return hardware_interface::return_type::OK;
    }
    
    bool valid_message_received = false;
    
    for (const auto& msg : messages) {
        auto parsed = MessageProtocol::parseMessage(msg);
        
        if (!parsed.valid) {
            diagnostics_->stats_.parse_errors++;
            RCLCPP_DEBUG_STREAM(logger_, "Failed to parse message: " << parsed.error);
            continue;
        }
        
        diagnostics_->stats_.messages_received++;
        valid_message_received = true;
        
        // Update states based on parsed message
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        for (size_t i = 0; i < joint_configs_.size(); i++) {
            std::string motor_id = std::string(1, 'a' + i);
            auto it = parsed.values.find(motor_id);
            
            if (it != parsed.values.end()) {
                velocity_states_[i] = it->second;
            }
        }
    }
    
    if (valid_message_received) {
        last_valid_message_ = std::chrono::steady_clock::now();
    }
    
    return hardware_interface::return_type::OK;
}

bool JennyBotInterface::sendCommands()
{
    if (!serial_manager_ || !serial_manager_->isConnected()) {
        return false;
    }
    
    // Prepare commands
    std::vector<std::pair<std::string, double>> commands;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < velocity_commands_.size() && i < joint_configs_.size(); i++) {
            std::string motor_id = std::string(1, 'a' + i);
            commands.push_back({motor_id, velocity_commands_[i]});
        }
    }
    
    // Create and send message
    std::string message = MessageProtocol::createCommand(commands);
    
    // Debug: log commands being sent
    static int log_counter = 0;
    if (log_counter++ % 100 == 0) {  // Log every 100 cycles to avoid spam
        std::stringstream ss;
        ss << "Sending commands: ";
        for (const auto& cmd : commands) {
            ss << cmd.first << "=" << cmd.second << " rad/s, ";
        }
        ss << "Message: " << message;
        RCLCPP_INFO(logger_, "%s", ss.str().c_str());
    }
    
    if (serial_manager_->writeMessage(message)) {
        diagnostics_->stats_.messages_sent++;
        return true;
    }
    
    return false;
}

}

PLUGINLIB_EXPORT_CLASS(jennybot_firmware::JennyBotInterface, hardware_interface::SystemInterface)