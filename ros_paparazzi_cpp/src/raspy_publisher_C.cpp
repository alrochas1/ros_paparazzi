// This is the node that will run on the raspberry
// Written in C++ for testing (not working yet)

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SondaSubscriber : public rclcpp::Node 
{
    public:
        SondaSubscriber() : Node("sonda_subscriber") {
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "sonda_status", 10, 
                std::bind(&SondaSubscriber::sonda_callback, this, std::placeholders::_1)
            );
        }

    private:
        void sonda_callback(const std_msgs::msg::String::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Estado de la sonda recibido: '%s'", msg->data.c_str());
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SondaSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
