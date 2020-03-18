#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <ros/topic.h>

#include "common.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_connector");
    ros::NodeHandle nh("~");

    std::string source_frame;
    if (!nh.getParam("source_frame", source_frame))
    {
        ROS_ERROR("source_frame param missing!");
        return -1;
    }
    std::string target_frame;
    if (!nh.getParam("target_frame", target_frame))
    {
        ROS_ERROR("target_frame param missing!");
        return -1;
    }
    std::string common_frame_wrt_source;
    if (!nh.getParam("common_wrt_source", common_frame_wrt_source))
    {
        ROS_ERROR("common_wrt_source param missing!");
        return -1;
    }
    std::string common_frame_wrt_target;
    if (!nh.getParam("common_wrt_target", common_frame_wrt_target))
    {
        ROS_ERROR("common_wrt_target param missing!");
        return -1;
    }

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    tf2_ros::StaticTransformBroadcaster source_to_target_broadcaster;

    // TODO: Use message filter to match timestamps
    // https://wiki.ros.org/tf2/Tutorials/Using%20stamped%20datatypes%20with%20tf2%3A%3AMessageFilter

    ros::Rate rate(10.0);
    while(nh.ok())
    {
        rate.sleep();
        auto now = ros::Time::now();
        tf2::Transform source_to_common_tf;
        try
        {
            geometry_msgs::Transform source_to_common_msg = tf_buffer.lookupTransform(source_frame, common_frame_wrt_source, now, ros::Duration(0.5)).transform;
            tf2::convert(source_to_common_msg, source_to_common_tf);
        } catch (tf2::TransformException &e)
        {
            ROS_WARN("%s", e.what());
            continue;
        }

        tf2::Transform target_to_common_tf;
        try {
            geometry_msgs::Transform target_to_common_msg = tf_buffer.lookupTransform(target_frame, common_frame_wrt_target, now, ros::Duration(0.5)).transform;
            tf2::convert(target_to_common_msg, target_to_common_tf);
        } catch (tf2::TransformException &e)
        {
            ROS_WARN("%s", e.what());
            continue;
        }

        tf2::Transform source_to_target_tf = source_to_common_tf * target_to_common_tf.inverse();
        geometry_msgs::TransformStamped source_to_target_msg;
        tf2::convert(source_to_target_tf, source_to_target_msg.transform);
        source_to_target_msg.header.stamp = now + ros::Duration(5.0);
        source_to_target_msg.header.frame_id = source_frame;
        source_to_target_msg.child_frame_id = target_frame;
        source_to_target_broadcaster.sendTransform(source_to_target_msg);
    }
}

