#include "ros/init.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <gnss_comm/rinex_helper.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <fstream>
#include <sys/stat.h>

using namespace gnss_comm;

bool file_exists(const std::string& path) {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
}

bool is_directory_writable(const std::string& path) {
    // 提取目录路径
    size_t pos = path.find_last_of("/\\");
    std::string dir = (pos != std::string::npos) ? path.substr(0, pos) : ".";
    
    // 检查目录是否可写
    return (access(dir.c_str(), W_OK) == 0);
}

std::vector<std::vector<ObsPtr>> parse_gnss_meas(const std::string &bag_filepath) {
    if (!file_exists(bag_filepath)) {
        ROS_ERROR("Input bag file does not exist: %s", bag_filepath.c_str());
        return {};
    }

    std::vector<std::vector<ObsPtr>> result;
    try {
        rosbag::Bag bag;
        bag.open(bag_filepath, rosbag::bagmode::Read);
        std::vector<std::string> topics = {"/ublox_driver/range_meas"};
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for(const auto& m : view) {
            GnssMeasMsgConstPtr obs_msg = m.instantiate<GnssMeasMsg>();
            if (obs_msg == NULL) 
			{
				ROS_WARN_STREAM("收到非 GnssMeasMsg 类型的消息，时间戳: " << m.getTime());
				continue;
			}
            std::vector<ObsPtr> obs = msg2meas(obs_msg);
            result.push_back(obs);
        }
    } catch (const rosbag::BagException& e) {
        ROS_ERROR("Error opening bag file: %s", e.what());
    }
    return result;
}

std::vector<EphemPtr> parse_gnss_ephem(const std::string &bag_filepath) {
    std::vector<EphemPtr> result;
    try {
        rosbag::Bag bag;
        bag.open(bag_filepath, rosbag::bagmode::Read);
        std::vector<std::string> topics = {"/ublox_driver/ephem"};
        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for(const auto& m : view) {
            GnssEphemMsgConstPtr ephem_msg = m.instantiate<GnssEphemMsg>();
            if (ephem_msg == NULL) continue;
            EphemPtr ephem = msg2ephem(ephem_msg);
            result.push_back(ephem);
        }
    } catch (const rosbag::BagException& e) {
        ROS_ERROR("Error opening bag file: %s", e.what());
    }
    return result;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bag2rinex");
    ros::NodeHandle nh("~");  // 使用私有命名空间
    
    // 获取参数
    std::string input_bag_path, output_obs_path, output_nav_path, output_gps_nav_path, output_bds_nav_path;
    bool obs_flag = true, nav_flag = true;

    nh.param<std::string>("input_bag_path", input_bag_path, "");
    nh.param<std::string>("output_obs_path", output_obs_path, "");
    nh.param<std::string>("output_nav_path", output_nav_path, "");
	nh.param<std::string>("output_gps_nav_path", output_gps_nav_path, "");
	nh.param<std::string>("output_bds_nav_path", output_bds_nav_path, "");
    nh.param<bool>("obs_flag", obs_flag, true);
    nh.param<bool>("nav_flag", nav_flag, true);

    // 验证参数
    if (input_bag_path.empty()) {
        ROS_ERROR("No input bag file specified!");
        return 1;
    }
    
    if (obs_flag && output_obs_path.empty()) {
        ROS_ERROR("Output OBS path not specified but obs_flag is true!");
        return 1;
    }
    
    if (nav_flag && output_nav_path.empty()) {
        ROS_ERROR("Output NAV path not specified but nav_flag is true!");
        return 1;
    }

    // 处理数据
    try {
        if (obs_flag) {
            if (!is_directory_writable(output_obs_path)) {
                ROS_ERROR("Cannot write to OBS output directory: %s", output_obs_path.c_str());
                return 1;
            }
			ROS_INFO_STREAM("begin to decode OBS data");
			auto all_gnss_meas = parse_gnss_meas(input_bag_path);
			ROS_INFO_STREAM("=== Done with OBS data decoded.. " << all_gnss_meas.size() << " size ===");
            if (!all_gnss_meas.empty()) {
                obs2rinex(output_obs_path, all_gnss_meas);
                ROS_INFO("Successfully wrote OBS data to: %s", output_obs_path.c_str());
            } else {
                ROS_WARN("No GNSS measurements found in the bag file");
            }
        }

        if (nav_flag) {
            if (!is_directory_writable(output_nav_path)) {
                ROS_ERROR("Cannot write to NAV output directory: %s", output_nav_path.c_str());
                return 1;
            }
			ROS_INFO_STREAM("begin to decode NAV data");
			auto all_gnss_ephem = parse_gnss_ephem(input_bag_path);
			ROS_INFO_STREAM("=== Done with NAV data decoded" << all_gnss_ephem.size() << " size ===");
            if (!all_gnss_ephem.empty()) {
                ephems2nav(output_nav_path, output_gps_nav_path, output_bds_nav_path, all_gnss_ephem);
                ROS_INFO("Successfully wrote NAV data to: %s", output_nav_path.c_str());
            } else {
                ROS_WARN("No GNSS ephemeris found in the bag file");
            }
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error during processing: %s", e.what());
        return 1;
    }

    ROS_INFO("Conversion completed successfully");
    return 0;
}