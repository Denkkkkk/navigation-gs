#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class CmdVelMultiplier : public rclcpp::Node
{
public:
    CmdVelMultiplier() : Node("cmd_vel_multiplier")
    {
        // 创建订阅者
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_controller", 10,
            std::bind(&CmdVelMultiplier::cmd_vel_callback, this, std::placeholders::_1));
        
        // 创建发布者
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "CmdVel multiplier node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: cmd_vel_controller");
        RCLCPP_INFO(this->get_logger(), "Publishing to: cmd_vel");
        RCLCPP_INFO(this->get_logger(), "Multiplying vx and az by 2.0");
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 创建新的Twist消息
        auto multiplied_msg = std::make_unique<geometry_msgs::msg::Twist>();
        
        // 复制所有原始数据
        *multiplied_msg = *msg;
        
        // 将vx和az乘以2倍
        multiplied_msg->linear.x = msg->linear.x * 2.0;
        multiplied_msg->angular.z = msg->angular.z * 1.0;
        
        // 发布处理后的消息
        publisher_->publish(std::move(multiplied_msg));
        
        // 可选：打印调试信息
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Original: vx=%.2f, az=%.2f | Multiplied: vx=%.2f, az=%.2f",
            msg->linear.x, msg->angular.z,
            multiplied_msg->linear.x, multiplied_msg->angular.z);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelMultiplier>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}