## ROS

### Must read
<https://answers.ros.org/question/58498/what-is-the-purpose-of-catkin_depends/>

### ROS Dependencies (Fedora)

    sudo dnf install --skip-broken python-empy console-bridge console-bridge-devel poco-devel boost boost-devel eigen3-devel pyqt4 qt-devel gcc gcc-c++ python-devel sip sip-devel tinyxml tinyxml-devel qt-devel qt5-devel python-qt5-devel sip sip-devel python3-sip python3-sip-devel qconf curl curl-devel gtest gtest-devel lz4-devel urdfdom-devel assimp-devel qhull-devel qhull uuid uuid-devel uuid-c++ uuid-c++-devel libuuid libuuid-devel gazebo gazebo-devel collada-dom collada-dom-devel yaml-cpp yaml-cpp-devel python2-defusedxml python-netifaces pyparsing pydot python-pyqtgraph python2-matplotlib

- `rqt_plot` breaks due to both PyQt4 and PyQt5
- do not use anaconda & pip to install missing packages

### [ROS `package.xml` rosdep key](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml)
<https://docs.ros.org/kinetic/api/catkin/html/howto/format1/system_library_dependencies.html>

### CMakeLists.txt for libraries
```
project(my_package_name) # Must match package name defined in package.xml
...
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_library_name # Must match library name defined in add_library(my_library_name ...)
)
...
add_library(my_library_name src/CountedTryFunc.cpp)
add_dependencies(my_library_name ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(my_library_name PUBLIC ${PROJECT_SOURCE_DIR}/include)
...
install(
  TARGETS
  my_library_name
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### CMakeLists.txt for header only libraries (CMake 3.0+ only)
<https://cmake.org/cmake/help/v3.2/manual/cmake-buildsystem.7.html#interface-libraries>
<http://www.mariobadr.com/creating-a-header-only-library-with-cmake.html>
<https://dominikberner.ch/cmake-interface-lib/>

Note that `colcon` automatically puts header files from `include` into `include/${PROJECT_NAME}`,
hence it is recommended to follow the convention for interoperability between `colcon` and `catkin`
```
include_directories(include)

catkin_package(
    INCLUDE_DIRS include
)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
```

### ROS2 header only library CMake
```
include(GNUInstallDirs)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${${PROJECT_NAME}_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

ament_export_interfaces(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
```

### ROS assumes robot is facing +x direction

### Build only some packages & dependencies
```
catkin_make --only-pkg-with-deps pkgs
```

### ROS interface code should be separate from main function / class code
Only bind relevant topics to relevant functions in the node main
```
using namespace std::placeholders;
MyClass my_class;
ros::Subscriber sub = n.subscribe("my_topic", std::bind([&](const std_msgs::Int8::ConstPtr& msg){
        my_class.my_func(msg->data);
    }, _1));
```

### Using functor with `subscribe`, `advertiseService`, `sendGoal`
```
sub = nh.subscribe<MyMessage>("my_topic", 1,
        [&](const MyMessage::ConstPtr& message)
        {
            ... *message ...
        });

service =  nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("my_service",
            [&](std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) -> bool
            {
                ...
            });

//https://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer(ExecuteCallbackMethod)
move_base.sendGoal(move_base_goal, [this](const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
        {
            ...
        });

```
Note that an empty capture `[]` is not allowed and will result in
```
call of overloaded 'subscribe(const char [20], int, main(int, char**)::<lambda(const ConstPtr&)>)' is ambiguous
```

### Build Targets
```
catkin_make tests # Build tests only
catkin_make run-tests # Build and run tests only
catkin_make all # All non-tests only
```

### Print tf from `/map` to `/base_link`
```
rosrun tf tf_echo /map /base_link
```

### Print current tf tree
```
rosrun rqt_tf_tree rqt_tf_tree
```

### RPY Specification
In ROS, roll, pitch, yaw are specified w.r.t. the initial frame.

### Linking libraries from another package
<https://answers.ros.org/question/240602/error-in-linking-a-catkin-library-against-another-catkin-package/>

### getMD5Sum error
```
/opt/ros/melodic/include/ros/message_traits.h:126:14: error: ‘const class std::__cxx11::basic_string<char>’ has no member named ‘__getMD5Sum’
     return m.__getMD5Sum().c_str();
```
Occurs when you
```
std::string msg = "...";
pub.publish(msg);
```
instead of
```
std_msgs::String msg;
msg.data = "...";
pub.publish(msg);
```

### Calling service in launch file
```
<node pkg="rosservice" type="rosservice" name="set_safezone" args="call --wait /set_safezone '{pose: {position: {x: 1.0, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, size: {x: 1.0, y: 1.0, z: 1.0}}'"/>
```

### Set log level from launch file
```
<node pkg="rosservice" type="rosservice" name="set_move_base_log_level" args="call --wait /move_base/set_logger_level 'ros.move_base' 'debug'" />
```

### `forming pointer to reference type` error
Check if function for subscription callback is fully defined, especially parameter type, i.e.
```
std::function<void(const my_msg_type::SharedPtr)> ...
```

### rviz plugins must have unique names in plugin xml
```
  <class name="my_plugin"
```

### Binding action server functions
```
using MyActionServer = actionlib::SimpleActionServer<my::MyAction>;

controller.registerPublishFeedback(
        [&](const my::MyFeedback& feedback)
        {
            action_server.publishFeedback(feedback);
        });
```
Alternative, uglier, `std::bind` way
```
// Because publishFeedback is overloaded
// https://stackoverflow.com/questions/13064698/stdbind-and-overloaded-function
controller.registerPublishFeedback(
        std::bind(static_cast<void(MyActionServer::*)(const my::MyFeedback&)>(&MyActionServer::publishFeedback), &action_server, std::placeholders::_1));
```
```
controller.registerSetSucceeded(
        [&](const my::MyResult& result)
        {
            action_server.setSucceeded(result);
        });
```
Alternative, uglier, `std::bind` way
```
controller.registerSetSucceeded(
        std::bind(&MyActionServer::setSucceeded, &action_server, std::placeholders::_1, ""));
```

### `ros::Time` and `ros::Rate` variables can never be global / static
<https://answers.ros.org/question/210252/terminate-called-after-throwing-an-instance-of-rostimenotinitializedexception-what-cannot-use-rostimenow-before-the-first-nodehandle-has-been-created/?answer=366122#post-id-366122i>

### `static_transform_broadcaster` in launch
```
<node pkg="tf2_ros" type="static_transform_publisher" name="my_tf" args="0 0 0 0 0 0 from_link to_link" />
```

### Actionlib GUI
Client:
```
rosrun actionlib axclient.py
```
Server:
```
rosrun actionlib axserver.py
```

### ROS Control
- <https://medium.com/@slaterobotics/how-to-implement-ros-control-on-a-custom-robot-748b52751f2e>
- <https://github.com/ros-controls/ros_control/wiki/hardware_interface>
- `controller_manager::update()` triggers all controllers associated with joints with names registered through `hardware_interface::JointStateInterface`
to read from variables registered through `hardware_interface::JointStateInterface`
and writes to the variables registered through `hardware_interface::XXXJointInterface`

### Linting
```
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright,-build/c++11,-whitespace/braces")
roslint_cpp()
set(ROSLINT_PYTHON_OPTS "1>&2") # Redirect stdout to stderr so it gets printed out during build
roslint_python()
roslint_add_test()
```

### Redirect `rostopic` output to file
```
stdbuf -oL rostopic hz /my_topic &> output.txt
```

### Troubleshooting TF lookup extrapolation errors
- Check relevant TF publishers publishing time
- Check relevant `TransformStamped` timestamps
- Check relevant TF buffer lookup time

## ROS2

#### ROS2 Control
- Currently, `JointTrajectoryController` only supports `hardware_interface::HW_IF_POSITION` (i.e. position command interface)

### Linking messages in same package
```
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/PointTrajectory.msg"
  LIBRARY_NAME ${PROJECT_NAME} # See https://github.com/ros2/rosidl/issues/441#issuecomment-591025515
)

add_library(${PROJECT_NAME} src/Robot.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  moveit_ros_planning_interface
)
rosidl_target_interfaces(${PROJECT_NAME} ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")

add_executable(node src/node.cpp)
ament_target_dependencies(node rclcpp)
target_link_libraries(node ${PROJECT_NAME})
rosidl_target_interfaces(node ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")

```

### `std_msgs` vs `builtin_interface`

### Troubleshooting `rosidl` errors for custom messages
- https://answers.ros.org/question/326008/ros2-run-symbol-not-found-on-custom-msg/
- Try cleaning `install` directory.  Sometimes `setup.zsh` is not updated correctly

### ROS2 latching
<https://answers.ros.org/question/305795/ros2-latching/>

### All `<link>`s in URDFs must have `<inertial>` to be used in simulation

### Disable line buffering when piping `ros2` commands
```
export PYTHONUNBUFFERED=1
ros2 topic echo ... | grep --line-buffered ...
```

## Docker

### `Failed to create an OpenGL context`
1. Set up `nvidia-docker` (https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
1. Add the following to the `.Dockerfile`
   ```
   ENV NVIDIA_VISIBLE_DEVICES \
     ${NVIDIA_VISIBLE_DEVICES:-all}
   ENV NVIDIA_DRIVER_CAPABILITIES \
     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
   ```
1. Rebuild image
1. Run image with `docker run --gpus all`

