#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief Subscriber node that logs messages from /chatter.
 */
class Listener : public rclcpp::Node
{
public:
  /**
   * @brief Construct a listener node and create the subscription.
   */
  Listener() : Node("listener")
  {
    RCLCPP_INFO(this->get_logger(), "Listener has started");
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/chatter",
      10,
      std::bind(&Listener::onMessage, this, std::placeholders::_1));
  }

private:
  /**
   * @brief Handle incoming chatter messages.
   * @param msg Received message.
   */
  void onMessage(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/**
 * @brief Entry point for the listener node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
