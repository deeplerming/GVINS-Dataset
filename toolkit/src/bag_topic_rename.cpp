#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "rosbag_topic_rename");

    // Create a ROS NodeHandle
    ros::NodeHandle nh;

    // Get input and output rosbag file paths from parameters
    std::string input_bag = nh.param<std::string>("input_bag", "input.bag");
    std::string output_bag = nh.param<std::string>("output_bag", "output.bag");

    // Create rosbag read and write objects
    rosbag::Bag in_bag;
    rosbag::Bag out_bag;

    try {
        // Open the input rosbag
        in_bag.open(input_bag, rosbag::bagmode::Read);

        // Open the output rosbag
        out_bag.open(output_bag, rosbag::bagmode::Write);

        // Retrieve the topic mapping from the parameter server
        std::map<std::string, std::string> topic_mapping;
        XmlRpc::XmlRpcValue topics;
        if (nh.getParam("topic_mapping", topics)) {
            for (int i = 0; i < topics.size(); i++) {
                std::string old_topic = static_cast<std::string>(topics[i]["old"]);
                std::string new_topic = static_cast<std::string>(topics[i]["new"]);
                topic_mapping[old_topic] = new_topic;
            }
        }

        // Iterate through all messages in the input rosbag
        rosbag::View view(in_bag);
        for (const rosbag::MessageInstance& m : view)
        {
            // Get the topic name and message type
            std::string topic = m.getTopic();

            // If the topic is in the mapping, change the topic name
            if (topic_mapping.find(topic) != topic_mapping.end()) {
                topic = topic_mapping[topic];
            }

            // Write the message to the new rosbag with the modified topic name
            out_bag.write(topic, m.getTime(), m);
        }

        // Close the rosbag files
        in_bag.close();
        out_bag.close();
    } catch (rosbag::BagException& e) {
        ROS_ERROR("Error: %s", e.what());
        return -1;
    }

    ROS_INFO("Rosbag file processing completed.");

    return 0;
}