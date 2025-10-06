#include <cstddef>
#include <array>
#include <cmath> // For round()
#include <vector> // For i2c_buffer in process_command
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cerrno>  // For errno
#include <cstring> // For strerror

// PCA9685 Registers
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06

class ServoController : public rclcpp::Node {
public:
    ServoController() : Node("servo_controller") {
        RCLCPP_INFO(this->get_logger(), "Servo controller node started!");

        // Initialize I2C
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "I2C init failed! Error: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "I2C initialized (FD: %d)", i2c_fd_);

        if (ioctl(i2c_fd_, I2C_SLAVE, 0x40) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set I2C slave address. Error: %s", strerror(errno));
            close(i2c_fd_);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "I2C slave address 0x40 set.");

        // Initialize PCA9685
        initialize_pca9685();

        // Initialize angles (e.g., to 90 degrees) and set initial servo positions
        angles_.fill(90); // Example: center all servos
        // set_initial_servo_positions(); // Optional: function to move all servos to initial angle

        // Subscription
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "keyboard_command", 10,
            [this](const std_msgs::msg::Int32MultiArray::ConstSharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received message with %zu elements", msg->data.size());
                if (msg->data.size() >= 2) {
                    RCLCPP_DEBUG(this->get_logger(), "Processing command: [%d, %d]",
                                 msg->data[0], msg->data[1]);
                    process_command(msg->data[0], msg->data[1]);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Malformed message received");
                }
            });

        RCLCPP_INFO(this->get_logger(), "Ready to receive commands on /keyboard_command");
    }

    ~ServoController() {
        if (i2c_fd_ >= 0) {
            // Optionally set servos to a safe state or turn off PWM output
            // For example, set MODE1 to sleep
            // write_i2c_byte(PCA9685_MODE1, 0x10); // Sleep
            close(i2c_fd_);
            RCLCPP_INFO(this->get_logger(), "I2C device closed.");
        }
    }

private:
    int i2c_fd_;
    std::array<int16_t, 13> angles_; // Assuming 13 servos (0-12)
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;

    // Writes a single byte to an I2C register
    void write_i2c_byte(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = {reg, value};
        if (write(i2c_fd_, buffer, 2) != 2) {
            RCLCPP_ERROR(this->get_logger(), "I2C write to reg 0x%02X failed! Error: %s", reg, strerror(errno));
        }
    }

    // Reads a single byte from an I2C register
    uint8_t read_i2c_byte(uint8_t reg) {
        uint8_t value = 0xFF; // Default error value
        if (write(i2c_fd_, &reg, 1) != 1) {
            RCLCPP_ERROR(this->get_logger(), "I2C read: failed to write register address 0x%02X. Error: %s", reg, strerror(errno));
            return value;
        }
        if (read(i2c_fd_, &value, 1) != 1) {
            RCLCPP_ERROR(this->get_logger(), "I2C read: failed to read value from reg 0x%02X. Error: %s", reg, strerror(errno));
            return 0xFF; // Indicate read failure
        }
        return value;
    }

    void initialize_pca9685() {
        RCLCPP_INFO(this->get_logger(), "Initializing PCA9685...");

        // Reset MODE1 to a known state (normal mode, but we'll modify it)
        write_i2c_byte(PCA9685_MODE1, 0x00);

        // Calculate prescaler for 300Hz (typical 25MHz internal oscillator)
        // prescale = round(osc_clock / (4096 * update_rate)) - 1
        // prescale = round(25,000,000 / (4096 * 300)) - 1 = round(20.345) - 1 = 20 - 1 = 19
        uint8_t prescale_val = 19; // For ~300Hz (actual: 305Hz)

        uint8_t old_mode1 = read_i2c_byte(PCA9685_MODE1);
        if (old_mode1 == 0xFF) { // Read error
             RCLCPP_ERROR(this->get_logger(), "Failed to read MODE1. PCA9685 initialization aborted.");
             return;
        }

        // Put PCA9685 to sleep to set prescaler
        uint8_t new_mode1 = (old_mode1 & 0x7F) | 0x10; // Set sleep bit (bit 4), clear restart (bit 7)
        write_i2c_byte(PCA9685_MODE1, new_mode1);
        write_i2c_byte(PCA9685_PRESCALE, prescale_val);

        // Wake up PCA9685
        // Restore old_mode1 but ensure sleep bit is cleared
        uint8_t mode1_awake = old_mode1 & ~0x10; // Clear sleep bit
        write_i2c_byte(PCA9685_MODE1, mode1_awake);

        usleep(500); // Wait for oscillator to stabilize (min 500µs)

        // Set Restart bit (bit 7) and Auto-Increment (AI bit 5)
        // It's important to set Restart after oscillator is running and stable.
        // Auto-Increment (AI) allows writing to subsequent registers automatically.
        write_i2c_byte(PCA9685_MODE1, mode1_awake | 0x80 | 0x20); // Set RESTART, Set AI

        RCLCPP_INFO(this->get_logger(), "PCA9685 initialized. MODE1: 0x%02X, Prescaler: %d (for ~300Hz)",
                    read_i2c_byte(PCA9685_MODE1), prescale_val);
    }

    // Optional: Function to set all servos to their initial angles stored in angles_
    // void set_initial_servo_positions() {
    //     RCLCPP_INFO(this->get_logger(), "Setting initial servo positions...");
    //     for (int i = 0; i < 13; ++i) { // Assuming 13 servos
    //         // This needs to call the PWM setting logic, similar to process_command
    //         // For simplicity, this is left as a conceptual step.
    //         // You would calculate pulse and value, then write to the servo's registers.
    //         // For now, process_command will handle setting them upon first keyboard input.
    //     }
    // }

    void process_command(int32_t servo_id, int32_t delta) {
        RCLCPP_INFO(this->get_logger(), "Command: Servo %d, Delta %d", servo_id, delta);

        if (servo_id < 0 || servo_id >= static_cast<int32_t>(angles_.size())) {
            RCLCPP_WARN(this->get_logger(), "Invalid servo ID: %d. Max ID: %zu", servo_id, angles_.size() -1);
            return;
        }

        int16_t new_angle = angles_[servo_id] + delta;
        if (new_angle < 0) new_angle = 0;
        else if (new_angle > 180) new_angle = 180;

        if (new_angle != angles_[servo_id]) {
            angles_[servo_id] = new_angle;

            // Convert angle to pulse width (1000-2000µs for typical 0-180 degree servos)
            uint16_t pulse_us = 1000 + (static_cast<uint32_t>(new_angle) * 1000 / 180);

            // Convert pulse width to PCA9685 ON/OFF register values
            // PCA9685 period = 1,000,000 µs / PWM_freq_Hz
            // For 300Hz, period = 1,000,000 / 300 = 3333.33 µs
            // For 305.17Hz (with prescale 19), period = 1,000,000 / 305.1757 = 3276.8 µs
            // PCA9685 counts from 0 to 4095
            // OFF_time_count = (pulse_us / period_us) * 4096
            // Using integer math for period: (1000000 / PWM_FREQ_INT_APPROX)
            uint16_t off_value = static_cast<uint16_t>((static_cast<uint32_t>(pulse_us) * 4096) / (1000000 / 300)); // Approx 300Hz
            // uint16_t off_value = static_cast<uint16_t>((static_cast<float>(pulse_us) / (1000000.0f / 305.1757f)) * 4095.0f); // More precise for 305Hz


            if (off_value > 4095) off_value = 4095;

            uint8_t start_reg = LED0_ON_L + 4 * servo_id;
            uint8_t i2c_payload[5];

            i2c_payload[0] = start_reg;    // Register address to start writing (LEDx_ON_L)
            i2c_payload[1] = 0x00;         // ON_L = 0 (pulse starts at count 0)
            i2c_payload[2] = 0x00;         // ON_H = 0
            i2c_payload[3] = static_cast<uint8_t>(off_value & 0xFF);       // OFF_L
            i2c_payload[4] = static_cast<uint8_t>((off_value >> 8) & 0xFF); // OFF_H

            if (write(i2c_fd_, i2c_payload, 5) != 5) {
                RCLCPP_ERROR(this->get_logger(), "PWM set failed for servo %d. Reg: 0x%02X. Error: %s",
                             servo_id, start_reg, strerror(errno));
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "Servo %d -> %d° (Pulse: %dµs, PCA_val: %d). Reg: 0x%02X",
                            servo_id, new_angle, pulse_us, off_value, start_reg);
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoController>();

    // Enable debug output if needed
    // auto ret = rcutils_logging_set_logger_level(
    //     node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    // if (ret != RCUTILS_RET_OK) {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to set logger level to DEBUG");
    // }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
