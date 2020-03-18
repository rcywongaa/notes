# Set model database location
```
export GAZEBO_MODEL_PATH=/home/user/catkin_ws/src/
```
or add the following in the `package.xml` of the package containing the models
```
  <export>
    <gazebo_ros gazebo_media_path="${prefix}"/>
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
  </export>
```


# `The goal pose passed to this planner must be in the map frame.  It is instead in the base_link frame.`
Remember to set the "Target Frame" to `map` in Rviz

# `Timed out waiting for transform from base_link to map to become available before running costmap`
Add following line to launch file
```
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
```

# Creating models for gazebo in blender
## Ensure objects are exported with the following settings
1. Export COLLADA (`.dae`)
   - Texture Options
     - UV Textures

## Recommended directory structure
```
models_package
├── models
│   ├── model1_name
│   │   ├── meshes
│   │   │   └── mesh_name.dae
│   │   ├── materials
│   │   │   └── textures
│   │   │       └── texture_name.png
```
1. Edit `.dae` file
   - Change path to texture to relative
   ```
   <init_from>../materials/textures/texture_name.png</init_from>
   ```

## Alternative directory structure
1. Export COLLADA (`.dae`)
   - Texture Options
     - UV Textures

```
models_package
├── models
│   ├── model1_name
│   │   ├── mesh_name.dae
│   │   ├── texture_name.png
```
1. Edit `.dae` file
   - Change path to texture to relative
   ```
   <init_from>texture_name.png</init_from>
   ```
   - Ensure `<ambient>` is set correctly for each material under `<phong>`
   ```
   <ambient>
     <color sid="ambient">0.9 0.9 0.9 1</color>
   </ambient>
   ```

# Setup differential drive, IMU, laser plugins (place in `.xacro` file)
Taken from <https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro>
```
  <gazebo>
    <plugin name="turtlebot3_burger_controller" filename="libgazebo_ros_diff_drive.so">
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <odometrySource>world</odometrySource>
      <publishOdomTF>true</publishOdomTF>
      <robotBaseFrame>base_footprint</robotBaseFrame>
      <publishWheelTF>false</publishWheelTF>
      <publishTf>true</publishTf>
      <publishWheelJointState>true</publishWheelJointState>
      <legacyMode>false</legacyMode>
      <updateRate>30</updateRate>
      <leftJoint>wheel_left_joint</leftJoint>
      <rightJoint>wheel_right_joint</rightJoint>
      <wheelSeparation>0.160</wheelSeparation>
      <wheelDiameter>0.066</wheelDiameter>
      <wheelAcceleration>1</wheelAcceleration>
      <wheelTorque>10</wheelTorque>
      <rosDebugLevel>na</rosDebugLevel>
    </plugin>
  </gazebo>

  <gazebo>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <alwaysOn>true</alwaysOn>
      <bodyName>imu_link</bodyName>
      <frameName>imu_link</frameName>
      <topicName>imu</topicName>
      <serviceName>imu_service</serviceName>
      <gaussianNoise>0.0</gaussianNoise>
      <updateRate>200</updateRate>
      <imu>
        <noise>
          <type>gaussian</type>
          <rate>
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </rate>
          <accel>
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </accel>
        </noise>
      </imu>
    </plugin>
  </gazebo>

  <gazebo reference="base_scan">
    <material>Gazebo/FlatBlack</material>
    <sensor type="ray" name="lds_lfcd_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>$(arg laser_visual)</visualize>
      <update_rate>5</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>6.28319</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.120</min>
          <max>3.5</max>
          <resolution>0.015</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_lds_lfcd_controller" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>base_scan</frameName>
      </plugin>
    </sensor>
  </gazebo>
```
