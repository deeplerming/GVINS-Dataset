#include "ros/node_handle.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_merge");
	ros::NodeHandle nh;
	
    std::string input_bag1, input_bag2, output_bag;
	
    nh.param<std::string>("input_bag_path_1", input_bag1, "");
    nh.param<std::string>("input_bag_path_2", input_bag2, "");
    nh.param<std::string>("output_bag_path", output_bag, "");
    
    if (argc != 4)
    {
        ROS_ERROR("Usage: rosbag_merge <input_bag1> <input_bag2> <output_bag>");
        return 1;
    }

    // 打开输入和输出的rosbag文件

    rosbag::Bag bag1, bag2, output_bag_file;
    
    try
    {
        // 打开两个输入文件
        bag1.open(input_bag1, rosbag::bagmode::Read);
        bag2.open(input_bag2, rosbag::bagmode::Read);
        
        // 打开输出文件
        output_bag_file.open(output_bag, rosbag::bagmode::Write);
        
        // 将第一个bag中的所有消息写入输出bag
        rosbag::View view1(bag1);
        BOOST_FOREACH(rosbag::MessageInstance const m, view1)
        {
            output_bag_file.write(m.getTopic(), m.getTime(), m);
        }

        // 将第二个bag中的所有消息写入输出bag
        rosbag::View view2(bag2);
        BOOST_FOREACH(rosbag::MessageInstance const m, view2)
        {
            output_bag_file.write(m.getTopic(), m.getTime(), m);
        }

        // 关闭文件
        bag1.close();
        bag2.close();
        output_bag_file.close();
        
        ROS_INFO("ROS bags merged successfully into: %s", output_bag.c_str());
    }
    catch (const rosbag::BagException& e)
    {
        ROS_ERROR("Error reading rosbag: %s", e.what());
        return 1;
    }
    
    return 0;
}
