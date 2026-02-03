#include <chrono>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief Publisher node that sends messages to /chatter.
 */
class Talker : public rclcpp::Node
{
public:
    /**
     * @brief Construct a talker node and create the publisher timer.
     */
    Talker()
        : Node("talker")
    {
        RCLCPP_INFO(this->get_logger(), "Talker has started");
        const auto topic = this->declare_parameter<std::string>("topic", "/chatter");
        const auto qos = rclcpp::QoS(10).reliable();
        const auto publish_period_ms =
            this->declare_parameter<int64_t>("publish_period_ms", 100);
        publisher_ = this->create_publisher<std_msgs::msg::String>(topic, qos);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms),
            std::bind(&Talker::onTimer, this));
    }

private:
    /**
     * @brief Publish a message on timer tick.
     */
    void onTimer()
    {
        std_msgs::msg::String msg;
        msg.data = "hello " + std::to_string(counter_++);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::size_t counter_{0};
};

/**
 * @brief Entry point for the talker node.
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}
