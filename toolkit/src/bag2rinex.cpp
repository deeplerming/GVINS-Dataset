#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <gnss_comm/rinex_helper.hpp>
#include <gnss_comm/gnss_ros.hpp>

#define INPUT_BAG_FILEPATH "/home/wxp/Downloads/bridge.bag"
#define OUTPUT_RINEX_FILEPATH "src/GVINS-Dataset/data/brige.ubx"

using namespace gnss_comm;

std::vector<std::vector<ObsPtr>> parse_gnss_meas(const std::string &bag_filepath)
{
    std::vector<std::vector<ObsPtr>> result;
    rosbag::Bag bag;
    bag.open(bag_filepath, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back("/ublox_driver/range_meas");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance const m : view)
    {
        GnssMeasMsgConstPtr obs_msg = m.instantiate<GnssMeasMsg>();
        if (obs_msg == NULL)
            continue;
        std::vector<ObsPtr> obs = msg2meas(obs_msg);
        result.push_back(obs);
    }
    return result;
}

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
    std::vector<std::vector<ObsPtr>> all_gnss_meas = parse_gnss_meas(INPUT_BAG_FILEPATH);
	std::vector<EphemPtr> all_gnss_ephem = parse_gnss_ephem(INPUT_BAG_FILEPATH);
    obs2rinex(OUTPUT_RINEX_FILEPATH, all_gnss_meas);
	ephems2nav(OUTPUT_RINEX_FILEPATH, all_gnss_ephem);
    std::cout << "Done\n";
    return 0;
}