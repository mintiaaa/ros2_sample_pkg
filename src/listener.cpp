#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Listener : public rclcpp::Node
{
public:
  Listener() : Node("listener")
  {
    RCLCPP_INFO(this->get_logger(), "Listener has started");
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/chatter",
      10,
      std::bind(&Listener::on_message, this, std::placeholders::_1));
  }

private:
  void on_message(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
