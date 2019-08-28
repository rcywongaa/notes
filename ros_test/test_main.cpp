#include <ros/ros.h>
#include "gtest/gtest.h"
#include <actionlib/TestAction.h>
#include <actionlib/client/simple_action_client.h>

const std::chrono::milliseconds WAIT_DURATION(500);

void wait()
{
    std::this_thread::sleep_for(WAIT_DURATION);
}

/* Action Clients */
std::unique_ptr<actionlib::SimpleActionClient<TestAction>> action_client;
TestActionResult action_result;
void onPayloadActionResult(const actionlib::SimpleClientGoalState &state,
                    const TestActionResult::ConstPtr &res)
{
	action_result.status = res->status;
}

/* Publishers */
ros::Publisher topic1_publisher;
void publishTopic1()
{
    topic1_publisher.publish(std_msgs::Empty());
}

/* Subscriber */
ros::Subscriber topic2_subscriber;
int topic2_msg;
void onTopic2(const std_msgs::Int8::ConstPtr& msg)
{
    topic2_msg = msg->data;
}

TEST(my_test, test)
{
    publishTopic1();
    wait();
    ASSERT_EQ(topic2_msg, 0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "my_ros_test_node");

    ros::NodeHandle n;
    // Initialize publishers and subscribers
    topic1_publisher = n.advertise<std_msgs::Empty>("topic1", 1, true);
    topic2_subscriber = n.subscribe<std_msgs::Int8>("topic2", 1, onTopic2);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    return ret;
}
