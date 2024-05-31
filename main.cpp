#include <memory>
#include <iostream>
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#include <filesystem>
#include "autonomous_proto.hpp"
static GeographicLib::LocalCartesian proj{};

struct navigation_date{
    std::vector<double> lat;
    std::vector<double> lon;
    std::vector<double> alt;
};
using namespace std;
autonomous_proto::Navigation nav;
template<typename MessageT>
MessageT deserialize_rosbag_message(std::shared_ptr<rosbag2_storage::SerializedBagMessage> &msg_ptr) {
    MessageT msg;
    rclcpp::SerializedMessage extracted_serialized_msg(*msg_ptr->serialized_data);
    rclcpp::Serialization<MessageT> serialization;
    serialization.deserialize_message(&extracted_serialized_msg, &msg);
    return msg;
}

void read_process_write_bag(const std::string& bag_file ,navigation_date& standard_path,navigation_date& needsolved_path ) {
    if (std::filesystem::exists("raw_bag_"+bag_file)) {
        std::filesystem::remove_all("raw_bag_"+bag_file);
    }
    auto node = rclcpp::Node::make_shared("rosbag2_reader_node");
    rosbag2_cpp::readers::SequentialReader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_file;
    storage_options.storage_id = "sqlite3";
    rosbag2_cpp::ConverterOptions converter_options;
    reader.open(storage_options, converter_options);
    std::string raw_bag_file = "raw_bag_"+bag_file ;
    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions output_storage_options;
    output_storage_options.uri = raw_bag_file;
    output_storage_options.storage_id = "sqlite3";
    writer.open(output_storage_options);
    RCLCPP_INFO(node->get_logger(),"rosbag2 : %s start parsing...",bag_file.c_str());

    while (reader.has_next()) {
        auto serialized_message = reader.read_next();
        auto topic_name = serialized_message->topic_name;
        auto time_stamp = serialized_message->time_stamp;
        auto data = serialized_message->serialized_data;
        auto target_topic = "/navigation";
        if (serialized_message->topic_name != target_topic) {
            continue;
        }
        if (topic_name == "/navigation") {
            auto msg = deserialize_rosbag_message<std_msgs::msg::UInt8MultiArray>(serialized_message);
            nav.ParseFromArray(msg.data.data(),msg.data.size());
            needsolved_path.lat.emplace_back(nav.position().lat().value());
            needsolved_path.lon.emplace_back(nav.position().lon().value());
            needsolved_path.alt.emplace_back(nav.position().alt().value());
        }
        proj.Reset(standard_path.lat[0], standard_path.lon[0], standard_path.lat[0]);
        static double e = 0, n = 0, u = 0;
        proj.Forward(nav.position().lat().value(),nav.position().lon().value(),nav.position().alt().value(), e , n , u );
//        RCLCPP_INFO(node->get_logger(),"Read message from topic: %s",serialized_message->topic_name.c_str());
//        RCLCPP_INFO(node->get_logger(), "Expected_path_lat data: %f ",nav.position().Expected_path_lat().value());
//        RCLCPP_INFO(node->get_logger(), "Expected_path_lon data: %f ",nav.position().Expected_path_lon().value());
//        RCLCPP_INFO(node->get_logger(), "Expected_path_alt data: %f ",nav.position().Expected_path_alt().value());
//        RCLCPP_INFO(node->get_logger(), "local_Cartesian_coordinates_e: %f ",e);
//        RCLCPP_INFO(node->get_logger(), "local_Cartesian_coordinates_n: %f ",n);
//        RCLCPP_INFO(node->get_logger(), "local_Cartesian_coordinates_u: %f ",u);
//        std::cout<<std::endl;
        auto e_topic_name ="/nav_e";
        auto n_topic_name ="/nav_n";
        auto u_topic_name ="/nav_u";

        std_msgs::msg::Float64 nav_e{},nav_n{}, nav_u{} ;
        nav_e.data = e;
        nav_n.data = n;
        nav_u.data = u;
        writer.write(nav_e,e_topic_name,(rclcpp::Time)time_stamp);
        writer.write(nav_n,n_topic_name,(rclcpp::Time)time_stamp);
        writer.write(nav_u,u_topic_name,(rclcpp::Time)time_stamp);
    }
    RCLCPP_INFO(node->get_logger(), "rosbag2 parsing successful! ");
    RCLCPP_INFO(node->get_logger(), "rosbag2 name: %s ",raw_bag_file.c_str());
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <Expected_path_bagfile> <Trace_path_bagfile>" << std::endl;
        return 1;
    }
    std::string Expected_path_bagfile = argv[1];
    std::string Trace_path_bagfile = argv[2];

    rclcpp::init(argc, argv);
    navigation_date Expected_path{};
    navigation_date Trace_path{};

    read_process_write_bag(Expected_path_bagfile, Expected_path, Expected_path);
    read_process_write_bag(Trace_path_bagfile, Expected_path, Trace_path);

    rclcpp::shutdown();
    return 0;
}