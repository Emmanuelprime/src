// ===================== JENNYBOT ESP32 FIRMWARE =====================
// Communication Protocol with ROS2 ros2_control hardware interface
// Message Format: $<motor_id><sign><velocity>,<motor_id><sign><velocity>*<checksum>\n
// Example: $a+005.23,b-003.14*4F\n
// motor_id: 'a'=left motor, 'b'=right motor
// velocity: rad/s (motor shaft velocity after gear reduction)
// checksum: XOR of all bytes between $ and *

#include <PID_v1.h>

// ===================== MOTOR PINS =====================
#define LEFT_SPD_PIN 16
#define LEFT_DIR1_PIN 17
#define LEFT_DIR2_PIN 18

#define RIGHT_SPD_PIN 25
#define RIGHT_DIR1_PIN 26
#define RIGHT_DIR2_PIN 27

// ===================== ENCODER PINS =====================
#define ENCODER_RIGHT_A 34
#define ENCODER_RIGHT_B 35
#define ENCODER_LEFT_A  32
#define ENCODER_LEFT_B  33

// ===================== ROBOT CONFIGURATION =====================
const float ENCODER_TICKS_PER_REV = 828.0;  // X4 decoding
const float ENCODER_CPR = 13.0;             // Encoder counts per revolution
const float GEAR_RATIO = 16.0;              // Motor:Wheel gear ratio
const float WHEEL_RADIUS = 0.042;           // meters
const int CONTROL_FREQUENCY = 50;           // Hz
const int CONTROL_PERIOD_MS = 1000 / CONTROL_FREQUENCY;  // 20ms

// PID Controller gains (tune these for your robot)
double KP = 50.0;
double KI = 5.0;
double KD = 0.5;

// ===================== GLOBAL VARIABLES =====================
volatile long right_encoder_count = 0;
volatile long left_encoder_count = 0;
volatile uint8_t right_last_state = 0;
volatile uint8_t left_last_state  = 0;

// Quadrature lookup table (X4 decoding)
const int8_t quad_table[16] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
 -1,  0,  0,  1,
  0,  1, -1,  0
};

// Velocity control variables
double left_target_velocity = 0.0;   // rad/s (motor shaft)
double right_target_velocity = 0.0;  // rad/s (motor shaft)
double left_current_velocity = 0.0;  // rad/s (motor shaft)
double right_current_velocity = 0.0; // rad/s (motor shaft)
double left_pwm_output = 0.0;        // PID output
double right_pwm_output = 0.0;       // PID output

// PID Controllers
PID leftPID(&left_current_velocity, &left_pwm_output, &left_target_velocity, KP, KI, KD, DIRECT);
PID rightPID(&right_current_velocity, &right_pwm_output, &right_target_velocity, KP, KI, KD, DIRECT);

// Timing
unsigned long last_control_time = 0;
unsigned long last_feedback_time = 0;
long last_left_encoder_count = 0;
long last_right_encoder_count = 0;

// Serial communication
String rx_buffer = "";
const int FEEDBACK_RATE_MS = 50;  // Send feedback at 20Hz

// Watchdog
unsigned long last_command_time = 0;
const int WATCHDOG_TIMEOUT_MS = 500;

// ===================== ENCODER ISR =====================
void IRAM_ATTR right_encoder_ISR() {
    uint8_t A = digitalRead(ENCODER_RIGHT_A);
    uint8_t B = digitalRead(ENCODER_RIGHT_B);
    uint8_t current_state = (A << 1) | B;
    uint8_t index = (right_last_state << 2) | current_state;
    right_encoder_count += quad_table[index];
    right_last_state = current_state;
}

void IRAM_ATTR left_encoder_ISR() {
    uint8_t A = digitalRead(ENCODER_LEFT_A);
    uint8_t B = digitalRead(ENCODER_LEFT_B);
    uint8_t current_state = (A << 1) | B;
    uint8_t index = (left_last_state << 2) | current_state;
    left_encoder_count -= quad_table[index];
    left_last_state = current_state;
}

// ===================== INITIALIZATION =====================
void initMotorPins() {
    pinMode(LEFT_SPD_PIN, OUTPUT);
    pinMode(LEFT_DIR1_PIN, OUTPUT);
    pinMode(LEFT_DIR2_PIN, OUTPUT);
    pinMode(RIGHT_SPD_PIN, OUTPUT);
    pinMode(RIGHT_DIR1_PIN, OUTPUT);
    pinMode(RIGHT_DIR2_PIN, OUTPUT);
    
    stopMotors();
}

void initEncoders() {
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);

    right_last_state = (digitalRead(ENCODER_RIGHT_A) << 1) | digitalRead(ENCODER_RIGHT_B);
    left_last_state  = (digitalRead(ENCODER_LEFT_A) << 1) | digitalRead(ENCODER_LEFT_B);

    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), right_encoder_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), right_encoder_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), left_encoder_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), left_encoder_ISR, CHANGE);
}

// ===================== MOTOR CONTROL =====================
void setMotorPWM(int leftPWM, int rightPWM) {
    // Constrain PWM values
    leftPWM = constrain(leftPWM, -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);
    
    // LEFT MOTOR
    if (leftPWM > 0) {
        digitalWrite(LEFT_DIR1_PIN, LOW);
        digitalWrite(LEFT_DIR2_PIN, HIGH);
    } else if (leftPWM < 0) {
        digitalWrite(LEFT_DIR1_PIN, HIGH);
        digitalWrite(LEFT_DIR2_PIN, LOW);
    } else {
        digitalWrite(LEFT_DIR1_PIN, HIGH);
        digitalWrite(LEFT_DIR2_PIN, HIGH); // brake
    }

    // RIGHT MOTOR
    if (rightPWM > 0) {
        digitalWrite(RIGHT_DIR1_PIN, LOW);
        digitalWrite(RIGHT_DIR2_PIN, HIGH);
    } else if (rightPWM < 0) {
        digitalWrite(RIGHT_DIR1_PIN, HIGH);
        digitalWrite(RIGHT_DIR2_PIN, LOW);
    } else {
        digitalWrite(RIGHT_DIR1_PIN, HIGH);
        digitalWrite(RIGHT_DIR2_PIN, HIGH); // brake
    }

    analogWrite(LEFT_SPD_PIN, abs(leftPWM));
    analogWrite(RIGHT_SPD_PIN, abs(rightPWM));
}

void stopMotors() {
    setMotorPWM(0, 0);
    left_target_velocity = 0.0;
    right_target_velocity = 0.0;
    leftPID.SetMode(MANUAL);
    rightPID.SetMode(MANUAL);
    left_pwm_output = 0.0;
    right_pwm_output = 0.0;
}

// ===================== VELOCITY CALCULATION =====================
void updateVelocities() {
    unsigned long current_time = millis();
    float dt = (current_time - last_control_time) / 1000.0; // seconds
    
    if (dt <= 0.0) return;
    
    // Read encoder counts safely
    noInterrupts();
    long current_left_count = left_encoder_count;
    long current_right_count = right_encoder_count;
    interrupts();
    
    // Calculate delta ticks
    long delta_left = current_left_count - last_left_encoder_count;
    long delta_right = current_right_count - last_right_encoder_count;
    
    // Convert ticks to motor shaft angular velocity (rad/s)
    // Motor shaft velocity = (delta_ticks / ENCODER_TICKS_PER_REV) * 2*PI / dt
    left_current_velocity = (delta_left / ENCODER_TICKS_PER_REV) * (2.0 * PI) / dt;
    right_current_velocity = (delta_right / ENCODER_TICKS_PER_REV) * (2.0 * PI) / dt;
    
    // Update for next iteration
    last_left_encoder_count = current_left_count;
    last_right_encoder_count = current_right_count;
    last_control_time = current_time;
}

// ===================== PID CONTROLLER =====================
void controlLoop() {
    unsigned long current_time = millis();
    
    if (current_time - last_control_time < CONTROL_PERIOD_MS) {
        return;
    }
    
    // Update current velocities
    updateVelocities();
    
    // Check watchdog
    if (current_time - last_command_time > WATCHDOG_TIMEOUT_MS) {
        stopMotors();
        return;
    }
    
    // Compute PID outputs
    leftPID.Compute();
    rightPID.Compute();
    
    // Apply PWM to motors
    setMotorPWM((int)left_pwm_output, (int)right_pwm_output);
}

// ===================== SERIAL PROTOCOL =====================
uint8_t calculateChecksum(String data) {
    uint8_t checksum = 0;
    for (int i = 0; i < data.length(); i++) {
        checksum ^= (uint8_t)data[i];
    }
    return checksum;
}

String createFeedbackMessage(float left_vel, float right_vel) {
    // Format: $a<sign><velocity>,b<sign><velocity>*<checksum>\n
    String msg = "";
    
    // Left motor (id='a')
    char left_sign = left_vel >= 0 ? '+' : '-';
    String left_val = String(abs(left_vel), 2);  // 2 decimal places
    while (left_val.length() < 6) left_val = "0" + left_val;  // pad to 6 chars
    msg += "a" + String(left_sign) + left_val;
    
    // Separator
    msg += ",";
    
    // Right motor (id='b')
    char right_sign = right_vel >= 0 ? '+' : '-';
    String right_val = String(abs(right_vel), 2);
    while (right_val.length() < 6) right_val = "0" + right_val;
    msg += "b" + String(right_sign) + right_val;
    
    // Calculate checksum
    uint8_t checksum = calculateChecksum(msg);
    char checksum_str[3];
    sprintf(checksum_str, "%02X", checksum);
    
    // Complete message
    return "$" + msg + "*" + String(checksum_str) + "\n";
}

bool parseCommandMessage(String msg) {
    // Validate format: $...*XX\n
    if (msg.length() < 5 || msg[0] != '$' || msg[msg.length()-1] != '\n') {
        return false;
    }
    
    // Find checksum marker
    int checksum_pos = msg.indexOf('*');
    if (checksum_pos == -1) {
        return false;
    }
    
    // Extract data and checksum
    String data = msg.substring(1, checksum_pos);
    String checksum_str = msg.substring(checksum_pos + 1, checksum_pos + 3);
    
    // Verify checksum
    uint8_t calculated = calculateChecksum(data);
    uint8_t received = strtol(checksum_str.c_str(), NULL, 16);
    
    if (calculated != received) {
        Serial.println("Checksum error!");
        return false;
    }
    
    // Parse motor commands
    // Expected format: a<sign><value>,b<sign><value>
    int comma_pos = data.indexOf(',');
    if (comma_pos == -1) {
        return false;
    }
    
    String left_cmd = data.substring(0, comma_pos);
    String right_cmd = data.substring(comma_pos + 1);
    
    // Parse left motor (id='a')
    if (left_cmd.length() >= 2 && left_cmd[0] == 'a') {
        char sign = left_cmd[1];
        float value = left_cmd.substring(2).toFloat();
        left_target_velocity = (sign == '+') ? value : -value;
    }
    
    // Parse right motor (id='b')
    if (right_cmd.length() >= 2 && right_cmd[0] == 'b') {
        char sign = right_cmd[1];
        float value = right_cmd.substring(2).toFloat();
        right_target_velocity = (sign == '+') ? value : -value;
    }
    
    last_command_time = millis();
    return true;
}

void processSerialInput() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n') {
            // Complete message received
            if (rx_buffer.length() > 0) {
                parseCommandMessage(rx_buffer + '\n');
                rx_buffer = "";
            }
        } else {
            rx_buffer += c;
            
            // Prevent buffer overflow
            if (rx_buffer.length() > 100) {
                rx_buffer = "";
            }
        }
    }
}

void sendFeedback() {
    unsigned long current_time = millis();
    
    if (current_time - last_feedback_time >= FEEDBACK_RATE_MS) {
        String feedback = createFeedbackMessage(left_current_velocity, right_current_velocity);
        Serial.print(feedback);
        last_feedback_time = current_time;
    }
}

// ===================== SETUP =====================
void setup() {
    Serial.begin(115200);  // USB serial for Pi communication
    
    initMotorPins();
    initEncoders();
    
    // Configure PID controllers
    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);
    leftPID.SetOutputLimits(-255, 255);
    rightPID.SetOutputLimits(-255, 255);
    leftPID.SetSampleTime(CONTROL_PERIOD_MS);
    rightPID.SetSampleTime(CONTROL_PERIOD_MS);
    
    last_control_time = millis();
    last_feedback_time = millis();
    last_command_time = millis();
    
    delay(1000);
    Serial.println("JennyBot ESP32 Firmware Ready");
    Serial.println("Waiting for commands...");
}

// ===================== MAIN LOOP =====================
void loop() {
    processSerialInput();
    controlLoop();
    sendFeedback();
}
