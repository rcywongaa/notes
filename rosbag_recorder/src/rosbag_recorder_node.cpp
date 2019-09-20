#include <rosbag/bag.h>
#include <rosbag/recorder.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "rosbag_recorder_node");
    // https://docs.ros.org/kinetic/api/rosbag/html/c++/structrosbag_1_1RecorderOptions.html
    rosbag::RecorderOptions recorder_options;
    recorder_options.topics.push_back("rosout");
    // Defaults save location of bags is ~/.ros
    rosbag::Recorder main_recorder(recorder_options);
    int result = main_recorder.run();
    ros::spin();
    return 0;
}
