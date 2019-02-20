#include <rclcpp/rclcpp.hpp>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>

class std_msgs_String_Subscriber
{
public:
    explicit std_msgs_String_Subscriber(const std::shared_ptr<rclcpp::Node> & node,
                                        topic_id_size_t serial_mapping,
                                        const std::string & name,
                                        std::shared_ptr<Transporter> transporter)
    {
        auto callback = [serial_mapping, transporter](const std_msgs::msg::String::SharedPtr msg) -> void
        {
            size_t headlen = transporter->get_header_length();
            char *data_buffer = new char[msg->data.size() + 5 + headlen]();
            eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[headlen], msg->data.size() + 5);
            eprosima::fastcdr::Cdr scdr(cdrbuffer);
            scdr << msg->data;
            transporter->write(serial_mapping, data_buffer, scdr.getSerializedDataLength());
            delete [] data_buffer;
        };
        sub = node->create_subscription<std_msgs::msg::String>(name, callback, rmw_qos_profile_default);
    }

private:
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub;
};
