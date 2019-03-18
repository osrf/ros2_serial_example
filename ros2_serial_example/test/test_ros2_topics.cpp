#include <gtest/gtest.h>

#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ros2_serial_example/transporter.hpp"
#include "ros2_topics.hpp"

class TransporterPassThrough : public ros2_to_serial_bridge::transport::Transporter
{
public:
    TransporterPassThrough(const std::string & protocol, size_t ring_buffer_size) : Transporter(protocol, ring_buffer_size)
    {
    }

    ~TransporterPassThrough() override
    {
    }

    ssize_t node_read() override
    {
        return 0;
    }

    ssize_t node_write(void *buffer, size_t len) override
    {
        (void)buffer;
        (void)len;
        return 0;
    }

    bool fds_OK() override
    {
        return true;
    }
};

class ROS2TopicsPassThrough : public ros2_to_serial_bridge::pubsub::ROS2Topics
{
public:
    explicit ROS2TopicsPassThrough(rclcpp::Node * node,
                                   const std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> & topic_names_and_serialization,
                                   ros2_to_serial_bridge::transport::Transporter * transporter) : ros2_to_serial_bridge::pubsub::ROS2Topics(node, topic_names_and_serialization, transporter)
    {
    }

    std::map<topic_id_size_t, std::unique_ptr<ros2_to_serial_bridge::pubsub::Publisher>> *get_serial_to_pub_map() const
    {
        return serial_to_pub_.get();
    }

    std::vector<std::unique_ptr<ros2_to_serial_bridge::pubsub::Subscription>> *get_serial_subs_vector() const
    {
        return serial_subs_.get();
    }
};

TEST(ROS2Topics, nullnode)
{
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);

    try
    {
        ros2_to_serial_bridge::pubsub::ROS2Topics r2(nullptr, topic_names_and_serialization, transporter.get());
    }
    catch (const std::runtime_error & err)
    {
        if (std::string(err.what()).find("Invalid node pointer") == std::string::npos)
        {
            FAIL() << "Expected error msg containing: Invalid node pointer" << std::endl
                   << "Saw error msg: " << std::endl
                   << err.what() << std::endl;
        }
    }
    catch (const std::exception & err)
    {
        FAIL() << "Expected exception of type " "std::runtime_error" << std::endl
               << "Saw exception of type: " << typeid(err).name() << std::endl;
    }
}

TEST(ROS2Topics, nulltransport)
{
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");

    try
    {
        ros2_to_serial_bridge::pubsub::ROS2Topics r2(node.get(), topic_names_and_serialization, nullptr);
    }
    catch (const std::runtime_error & err)
    {
        if (std::string(err.what()).find("Invalid transporter pointer") == std::string::npos)
        {
            FAIL() << "Expected error msg containing: Invalid transporter pointer" << std::endl
                   << "Saw error msg: " << std::endl
                   << err.what() << std::endl;
        }
    }
    catch (const std::exception & err)
    {
        FAIL() << "Expected exception of type " "std::runtime_error" << std::endl
               << "Saw exception of type: " << typeid(err).name() << std::endl;
    }
}

TEST(ROS2Topics, duplicate_pub_mapping)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    topic_names_and_serialization["foo"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["foo"].serial_mapping = 9;
    topic_names_and_serialization["foo"].type = "std_msgs/String";
    topic_names_and_serialization["foo"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::SERIAL_TO_ROS2;

    topic_names_and_serialization["bar"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["bar"].serial_mapping = 9;
    topic_names_and_serialization["bar"].type = "std_msgs/String";
    topic_names_and_serialization["bar"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::SERIAL_TO_ROS2;

    try
    {
        ros2_to_serial_bridge::pubsub::ROS2Topics r2(node.get(), topic_names_and_serialization, transporter.get());
    }
    catch (const std::runtime_error & err)
    {
        if (std::string(err.what()).find("has duplicate pub serial_mapping") == std::string::npos)
        {
            FAIL() << "Expected error msg containing: has duplicate pub serial_mapping" << std::endl
                   << "Saw error msg: " << std::endl
                   << err.what() << std::endl;
        }
    }
    catch (const std::exception & err)
    {
        FAIL() << "Expected exception of type " "std::runtime_error" << std::endl
               << "Saw exception of type: " << typeid(err).name() << std::endl;
    }
}

TEST(ROS2Topics, duplicate_sub_mapping)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    topic_names_and_serialization["foo"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["foo"].serial_mapping = 9;
    topic_names_and_serialization["foo"].type = "std_msgs/String";
    topic_names_and_serialization["foo"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::ROS2_TO_SERIAL;

    topic_names_and_serialization["bar"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["bar"].serial_mapping = 9;
    topic_names_and_serialization["bar"].type = "std_msgs/String";
    topic_names_and_serialization["bar"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::ROS2_TO_SERIAL;

    try
    {
        ros2_to_serial_bridge::pubsub::ROS2Topics r2(node.get(), topic_names_and_serialization, transporter.get());
    }
    catch (const std::runtime_error & err)
    {
        if (std::string(err.what()).find("has duplicate sub serial_mapping") == std::string::npos)
        {
            FAIL() << "Expected error msg containing: has duplicate sub serial_mapping" << std::endl
                   << "Saw error msg: " << std::endl
                   << err.what() << std::endl;
        }
    }
    catch (const std::exception & err)
    {
        FAIL() << "Expected exception of type " "std::runtime_error" << std::endl
               << "Saw exception of type: " << typeid(err).name() << std::endl;
    }
}

TEST(ROS2Topics, invalid_type)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    topic_names_and_serialization["foo"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["foo"].serial_mapping = 9;
    topic_names_and_serialization["foo"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::SERIAL_TO_ROS2;

    ROS2TopicsPassThrough r2(node.get(), topic_names_and_serialization, transporter.get());

    std::map<topic_id_size_t, std::unique_ptr<ros2_to_serial_bridge::pubsub::Publisher>> *serial_to_pub = r2.get_serial_to_pub_map();
    ASSERT_EQ(serial_to_pub->size(), 0U);

    std::vector<std::unique_ptr<ros2_to_serial_bridge::pubsub::Subscription>> *serial_subs = r2.get_serial_subs_vector();
    ASSERT_EQ(serial_subs->size(), 0U);
}

TEST(ROS2Topics, invalid_serial_mapping)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    topic_names_and_serialization["foo"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["foo"].type = "std_msgs/String";
    topic_names_and_serialization["foo"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::SERIAL_TO_ROS2;

    ROS2TopicsPassThrough r2(node.get(), topic_names_and_serialization, transporter.get());

    std::map<topic_id_size_t, std::unique_ptr<ros2_to_serial_bridge::pubsub::Publisher>> *serial_to_pub = r2.get_serial_to_pub_map();
    ASSERT_EQ(serial_to_pub->size(), 0U);

    std::vector<std::unique_ptr<ros2_to_serial_bridge::pubsub::Subscription>> *serial_subs = r2.get_serial_subs_vector();
    ASSERT_EQ(serial_subs->size(), 0U);
}

TEST(ROS2Topics, invalid_direction)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    topic_names_and_serialization["foo"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["foo"].serial_mapping = 9;
    topic_names_and_serialization["foo"].type = "std_msgs/String";

    ROS2TopicsPassThrough r2(node.get(), topic_names_and_serialization, transporter.get());

    std::map<topic_id_size_t, std::unique_ptr<ros2_to_serial_bridge::pubsub::Publisher>> *serial_to_pub = r2.get_serial_to_pub_map();
    ASSERT_EQ(serial_to_pub->size(), 0U);

    std::vector<std::unique_ptr<ros2_to_serial_bridge::pubsub::Subscription>> *serial_subs = r2.get_serial_subs_vector();
    ASSERT_EQ(serial_subs->size(), 0U);
}

TEST(ROS2Topics, reserved_serial_mapping0)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    topic_names_and_serialization["foo"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["foo"].serial_mapping = 0;
    topic_names_and_serialization["foo"].type = "std_msgs/String";
    topic_names_and_serialization["foo"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::SERIAL_TO_ROS2;

    ROS2TopicsPassThrough r2(node.get(), topic_names_and_serialization, transporter.get());

    std::map<topic_id_size_t, std::unique_ptr<ros2_to_serial_bridge::pubsub::Publisher>> *serial_to_pub = r2.get_serial_to_pub_map();
    ASSERT_EQ(serial_to_pub->size(), 0U);

    std::vector<std::unique_ptr<ros2_to_serial_bridge::pubsub::Subscription>> *serial_subs = r2.get_serial_subs_vector();
    ASSERT_EQ(serial_subs->size(), 0U);
}

TEST(ROS2Topics, reserved_serial_mapping1)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    topic_names_and_serialization["foo"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["foo"].serial_mapping = 1;
    topic_names_and_serialization["foo"].type = "std_msgs/String";
    topic_names_and_serialization["foo"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::SERIAL_TO_ROS2;

    ROS2TopicsPassThrough r2(node.get(), topic_names_and_serialization, transporter.get());

    std::map<topic_id_size_t, std::unique_ptr<ros2_to_serial_bridge::pubsub::Publisher>> *serial_to_pub = r2.get_serial_to_pub_map();
    ASSERT_EQ(serial_to_pub->size(), 0U);

    std::vector<std::unique_ptr<ros2_to_serial_bridge::pubsub::Subscription>> *serial_subs = r2.get_serial_subs_vector();
    ASSERT_EQ(serial_subs->size(), 0U);
}

TEST(ROS2Topics, serial_mapping_too_large)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    topic_names_and_serialization["foo"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["foo"].serial_mapping = std::numeric_limits<topic_id_size_t>::max() + 1;
    topic_names_and_serialization["foo"].type = "std_msgs/String";
    topic_names_and_serialization["foo"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::SERIAL_TO_ROS2;

    ROS2TopicsPassThrough r2(node.get(), topic_names_and_serialization, transporter.get());

    std::map<topic_id_size_t, std::unique_ptr<ros2_to_serial_bridge::pubsub::Publisher>> *serial_to_pub = r2.get_serial_to_pub_map();
    ASSERT_EQ(serial_to_pub->size(), 0U);

    std::vector<std::unique_ptr<ros2_to_serial_bridge::pubsub::Subscription>> *serial_subs = r2.get_serial_subs_vector();
    ASSERT_EQ(serial_subs->size(), 0U);
}

TEST(ROS2Topics, unsupported_pub_type)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    topic_names_and_serialization["foo"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["foo"].serial_mapping = 9;
    topic_names_and_serialization["foo"].type = "invalid_test_msgs/String";
    topic_names_and_serialization["foo"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::SERIAL_TO_ROS2;

    ROS2TopicsPassThrough r2(node.get(), topic_names_and_serialization, transporter.get());

    std::map<topic_id_size_t, std::unique_ptr<ros2_to_serial_bridge::pubsub::Publisher>> *serial_to_pub = r2.get_serial_to_pub_map();
    ASSERT_EQ(serial_to_pub->size(), 0U);

    std::vector<std::unique_ptr<ros2_to_serial_bridge::pubsub::Subscription>> *serial_subs = r2.get_serial_subs_vector();
    ASSERT_EQ(serial_subs->size(), 0U);
}

TEST(ROS2Topics, unsupported_sub_type)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    topic_names_and_serialization["foo"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["foo"].serial_mapping = 9;
    topic_names_and_serialization["foo"].type = "invalid_test_msgs/String";
    topic_names_and_serialization["foo"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::ROS2_TO_SERIAL;

    ROS2TopicsPassThrough r2(node.get(), topic_names_and_serialization, transporter.get());

    std::map<topic_id_size_t, std::unique_ptr<ros2_to_serial_bridge::pubsub::Publisher>> *serial_to_pub = r2.get_serial_to_pub_map();
    ASSERT_EQ(serial_to_pub->size(), 0U);

    std::vector<std::unique_ptr<ros2_to_serial_bridge::pubsub::Subscription>> *serial_subs = r2.get_serial_subs_vector();
    ASSERT_EQ(serial_subs->size(), 0U);
}

TEST(ROS2Topics, one_pub_mapping)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    topic_names_and_serialization["foo"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["foo"].serial_mapping = 9;
    topic_names_and_serialization["foo"].type = "std_msgs/String";
    topic_names_and_serialization["foo"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::SERIAL_TO_ROS2;

    ROS2TopicsPassThrough r2(node.get(), topic_names_and_serialization, transporter.get());

    std::map<topic_id_size_t, std::unique_ptr<ros2_to_serial_bridge::pubsub::Publisher>> *serial_to_pub = r2.get_serial_to_pub_map();
    ASSERT_EQ(serial_to_pub->size(), 1U);

    std::vector<std::unique_ptr<ros2_to_serial_bridge::pubsub::Subscription>> *serial_subs = r2.get_serial_subs_vector();
    ASSERT_EQ(serial_subs->size(), 0U);
}

TEST(ROS2Topics, one_sub_mapping)
{
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node");
    std::unique_ptr<TransporterPassThrough> transporter = std::make_unique<TransporterPassThrough>("px4", 8192);
    std::map<std::string, ros2_to_serial_bridge::pubsub::TopicMapping> topic_names_and_serialization;

    topic_names_and_serialization["bar"] = ros2_to_serial_bridge::pubsub::TopicMapping();
    topic_names_and_serialization["bar"].serial_mapping = 9;
    topic_names_and_serialization["bar"].type = "std_msgs/String";
    topic_names_and_serialization["bar"].direction = ros2_to_serial_bridge::pubsub::TopicMapping::Direction::ROS2_TO_SERIAL;

    ROS2TopicsPassThrough r2(node.get(), topic_names_and_serialization, transporter.get());

    std::map<topic_id_size_t, std::unique_ptr<ros2_to_serial_bridge::pubsub::Publisher>> *serial_to_pub = r2.get_serial_to_pub_map();
    ASSERT_EQ(serial_to_pub->size(), 0U);

    std::vector<std::unique_ptr<ros2_to_serial_bridge::pubsub::Subscription>> *serial_subs = r2.get_serial_subs_vector();
    ASSERT_EQ(serial_subs->size(), 1U);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    return RUN_ALL_TESTS();
}
