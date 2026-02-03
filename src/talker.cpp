#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Talker: public rclcpp::Node
{
public:
    Talker()
        : Node("talker")
    {
        RCLCPP_INFO(this->get_logger(), "Talker has started");
        const auto topic = this->declare_parameter<std::string>("topic", "/chatter");
        publisher_ = this->create_publisher<std_msgs::msg::String>(topic, 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Talker::on_timer, this));
    }

private:
    void on_timer()
    {
        std_msgs::msg::String msg;
        msg.data = "hello";
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}
