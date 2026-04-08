#include <csignal>
#include <memory>
#include <vector>
#include <string>

#include <unistd.h>
#include <rclcpp/rclcpp.hpp>

#include "common/glog_utils.hpp"
#include "common/timer_utils.hpp"
#include "interface/ros2/ros_interface.h"

DEFINE_string(config_file, "config/leg_fusion.yaml", "Path to the YAML file");

void sigHandle(int sig) {
    legkilo::options::FLAG_EXIT.store(true);
    LOG(INFO) << "catch sig " << sig << "  FLAG_EXIT = True";
}

int main(int argc, char** argv) {
    // ROS 2 initialization with original arguments
    rclcpp::init(argc, argv);

    // Use ROS 2's built-in function to remove ROS-specific arguments
    // This gives us clean arguments for gflags parsing
    std::vector<std::string> filtered_args = rclcpp::remove_ros_arguments(argc, argv);
    
    // Convert back to char** for gflags
    std::vector<char*> argv_vec;
    for (auto& arg : filtered_args) {
        argv_vec.push_back(const_cast<char*>(arg.c_str()));
    }
    int argc_filtered = static_cast<int>(argv_vec.size());
    char** argv_filtered = argv_vec.data();

    signal(SIGINT, sigHandle);

    google::ParseCommandLineFlags(&argc_filtered, &argv_filtered, true);

    if (FLAGS_config_file.empty()) {
        std::cerr << "YAML configuration file path not provided. Use --config_file=<path>." << std::endl;
        return -1;
    }

    const std::string root_dir = legkilo::resolveRootDirFromConfigFile(FLAGS_config_file);

    // Logging uses the resolved root_dir so output follows YAML config when provided.
    std::unique_ptr<legkilo::Logging> logging(new legkilo::Logging(argc_filtered, argv_filtered, "logs", root_dir));
    
    // ROS 2: Create node with NodeOptions
    auto ros_interface_node = std::make_shared<legkilo::RosInterface>(rclcpp::NodeOptions());

    // ROS 2: Use init() instead of rosInit()
    ros_interface_node->init(FLAGS_config_file);

    LOG(INFO) << "Leg KILO Node Starts";

    // ROS 2: Main loop with rclcpp::Rate
    rclcpp::Rate rate(5000);
    while (rclcpp::ok() && !legkilo::options::FLAG_EXIT.load()) {
        ros_interface_node->run();
        rate.sleep();
    }
    legkilo::options::FLAG_EXIT.store(true);

    // Explicitly reset ros_interface_node to ensure proper cleanup
    ros_interface_node.reset();
    LOG(INFO) << "RosInterface destroyed";
    LOG(INFO) << "Leg KILO Node Ends";
    legkilo::Timer::logAllAverTime();
    logging->flushLogFiles();
    
    // ROS 2 shutdown
    rclcpp::shutdown();
    return 0;
}
