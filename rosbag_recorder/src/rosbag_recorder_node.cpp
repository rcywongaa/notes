#include <rosbag/bag.h>
#include <rosbag/recorder.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosbag_recorder_node");
    ros::NodeHandle nh("~");

    // https://docs.ros.org/kinetic/api/rosbag/html/c++/structrosbag_1_1RecorderOptions.html
    rosbag::RecorderOptions recorder_options;

    int max_size_G = 10;
    nh.param("max_size_G", max_size_G, 10); // 10GB
    recorder_options.max_size = (uint64_t)max_size_G*1024*1024*1024;
    recorder_options.split = true;
    int max_splits = 10;
    nh.param("max_splits", max_splits, 10);
    recorder_options.max_splits = (uint32_t)max_splits;
    int min_space_G = 100;
    nh.param("min_space_G", min_space_G, 100);
    recorder_options.min_space = (unsigned long long)min_space_G*1024*1024*1024; // 500GB
    recorder_options.min_space_str = std::to_string(min_space_G) + "G";

    int buffer_size_M = 2*1024;
    nh.param("buffer_size_M", buffer_size_M, 1);
    recorder_options.buffer_size = (uint32_t)buffer_size_M*1024*1024; // 2GB
    std::vector<std::string> topics;
    nh.getParam("topics", topics);
    ROS_INFO("Num topics: %ld", topics.size());
    for(auto topic : topics)
    {
        ROS_INFO("Bagging topic: %s", topic.c_str());
        recorder_options.topics.push_back(topic);
    }
    // Defaults save location of bags is ~/.ros
    rosbag::Recorder main_recorder(recorder_options);
    int result = main_recorder.run();
    ros::spin();
    return 0;
}


