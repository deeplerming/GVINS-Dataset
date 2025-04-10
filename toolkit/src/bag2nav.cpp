#include "gnss_constant.hpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <gnss_comm/rinex_helper.hpp>
#include <gnss_comm/gnss_ros.hpp>

#define INPUT_BAG_FILEPATH "/home/wxp/Downloads/bridge.bag"
#define OUTPUT_RINEX_FILEPATH "src/GVINS-Dataset/data/brige.ubx"

using namespace gnss_comm;

std::vector<EphemPtr> parse_gnss_ephem(const std::string &bag_filepath)
{
    std::vector<EphemPtr> result;
    rosbag::Bag bag;
    bag.open(bag_filepath, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back("/ublox_driver/ephem");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance const m : view)
    {
        GnssEphemMsgConstPtr ephem_msg = m.instantiate<GnssEphemMsg>();
        if (ephem_msg == NULL)
            continue;
        EphemPtr ephem = msg2ephem(ephem_msg);
        result.push_back(ephem);
    }
    return result;
}

int main(int argc, char **argv)
{
    std::vector<EphemPtr> all_gnss_ephem = parse_gnss_ephem(INPUT_BAG_FILEPATH);
    ephems2nav(OUTPUT_RINEX_FILEPATH, all_gnss_ephem);
    std::cout << "Done\n";
    return 0;
}