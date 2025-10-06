#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"  // Changed from Int16MultiArray
#include <termios.h>
#include <unistd.h>
#include <map>
#include <iostream>

class KeyboardNode : public rclcpp::Node {
public:
    KeyboardNode() : Node("keyboard_node") {
        // Changed to Int32MultiArray
        pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("keyboard_command", 10);
        setup_keymap();
        RCLCPP_INFO(get_logger(), "Press keys (a-z) to control servos. ESC to quit.");
        timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { read_key(); });
    }

private:
    // Changed to Int32MultiArray
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::map<char, std::pair<int16_t, int16_t>> keymap_;

    void setup_keymap() {
        char key = 'a';
        for (int servo = 0; servo < 13; servo++) {
            keymap_[key++] = {servo, +5};
            keymap_[key++] = {servo, -5};
        }
    }

    void read_key() {
        char key = getch();
        if (key == 27) {  // ESC key
            RCLCPP_INFO(get_logger(), "Shutting down...");
            rclcpp::shutdown();
            return;
        }

        if (keymap_.count(key)) {
            auto [servo_id, delta] = keymap_[key];
            auto msg = std_msgs::msg::Int32MultiArray();  // Changed to Int32
            msg.data = {static_cast<int32_t>(servo_id), static_cast<int32_t>(delta)};  // Explicit casting
            pub_->publish(msg);

            std::cout << "Pressed: '" << key << "' → Servo " << servo_id 
                      << (delta > 0 ? " +" : " ") << delta << "°\n";
        } else if (isprint(key)) {
            std::cout << "Pressed invalid key: '" << key << "' (a-z only)\n";
        }
    }

    char getch() {
        char buf = 0;
        struct termios old = {};
        tcgetattr(0, &old);
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        tcsetattr(0, TCSANOW, &old);
        read(0, &buf, 1);
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        tcsetattr(0, TCSANOW, &old);
        return buf;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardNode>());
    rclcpp::shutdown();
    return 0;
}
