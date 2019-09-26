# markerLocalization
Localize a robot using aruco markers and realsence cameras

# Dependencies

ros-kinetic-ar-track-alvar
ros-kinetic-raspicam-node

# for the burgers


```BASH
rosed turtlebot3_description turbot3_burger.urdf.xacro

```
add the following lines at the end: 


```XML
<joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.058 0 0.129" rpy="0 1.570796325 3.14159265359"/> <!-- p was 1.57.. -->
  </joint>
  <!-- 3.14159265359 -->
  <!-- 1.570796325 -->
  <!-- THE PARAMETERS BELLOW ARE NOT RIGHT!!!! NEEDS MORE CONSIDERATION -->
  <link name="camera_link">
    <!--visual>
      <origin xyz="0 0 0.0" rpy="0 0 0"/>
      <-geometry>
        <mesh filename="package://turtlebot3_description/meshes/sensors/lds.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="dark"/>
    </visual-->

    <collision>
      <origin xyz="0.015 0 -0.0065" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.0315" radius="0.055"/>
      </geometry>
    </collision>

    <inertial>
    <mass value="0.114" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0"
               izz="0.001" />
    </inertial>

```

this is to set up the transform between the base link and the raspi camera

```BASH
rosed raspicam_node camerav2_1280x960_10fps.launch
```
and change the camera frame from raspicam to camera link at line 4

```XML
<arg name="camera_frame_id" default="camera_link"/>
```
and somewhere the param:

```XML
<param name="/raspicam_node/quality"  value="9"/>
```


```BASH
cd /home/robot/.ros/
mkdir camera_info
```
then move to the raspicam node package

```BASH
roscd raspicam_node/camera_info/
cp camerav2_1280x960.yaml /home/robot/.ros/camera_info/
```

Also remember to choose the right marker size
and it should work



