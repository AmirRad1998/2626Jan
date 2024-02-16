Answer 1


<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot3_xacro">
<xacro:include filename="$(find robot_urdf)/urdf/robot3.gazebo"/>

<xacro:property name="length_wheel" value="0.04" />
<xacro:property name="radius_wheel" value="0.1" />

<xacro:macro name="default_inertial" params="mass">
    <inertial>
        <mass value="${mass}" />
        <origin rpy="0 1.5707 1.5707" xyz="0 0 0"/>
        <inertia ixx="0.000526666666667" ixy="0" ixz="0" iyy="0.000526666666667" iyz="0" izz="0.001"/>
    </inertial>
</xacro:macro>

<xacro:macro name="wheel_geometry">
    <geometry>
        <cylinder length="${length_wheel}" radius="${radius_wheel}"/>
    </geometry>
</xacro:macro>

<!-- Material definitions remain unchanged -->

<link name="link_chassis">
    <!-- pose and inertial -->
    <pose>0 0 0.1 0 0 0</pose>
    <inertial>
      <mass value="5"/>
      <origin rpy="0 0 0" xyz="0 0 0.1"/>
      <inertia ixx="0.0395416666667" ixy="0" ixz="0" iyy="0.106208333333" iyz="0" izz="0.106208333333"/>
    </inertial>
    <!-- body -->
    <collision name="collision_chassis">
      <geometry>
        <box size="0.5 0.3 0.07"/>
      </geometry>
    </collision>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <box size="0.5 0.3 0.07"/>
      </geometry>
      <material name="blue"/>
    </visual>
</link>

<!-- Duplicate and adjust for additional wheels -->
<!-- Original Right Wheel -->
<link name="link_right_wheel">
    <xacro:default_inertial mass="0.2"/>
    <collision name="link_right_wheel_collision">
      <origin rpy="0 1.5707 1.5707" xyz="0 0 0"/>
      <xacro:wheel_geometry />
    </collision>
    <visual name="link_right_wheel_visual">
      <origin rpy="0 1.5707 1.5707" xyz="0 0 0"/>
      <xacro:wheel_geometry />
      <material name="red"/>
    </visual>
</link>
<joint name="joint_right_wheel" type="continuous">
    <origin rpy="0 0 0" xyz="0.25 0.15 0"/>
    <child link="link_right_wheel"/>
    <parent link="link_chassis"/>
    <axis rpy="0 0 0" xyz="0 1 0"/>
</joint>

<!-- Original Left Wheel -->
<link name="link_left_wheel">
    <xacro:default_inertial mass="0.2"/>
    <collision name="link_left_wheel_collision">
      <origin rpy="0 1.5707 1.5707" xyz="0 0 0"/>
      <xacro:wheel_geometry />
    </collision>
    <visual name="link_left_wheel_visual">
      <origin rpy="0 1.5707 1.5707" xyz="0 0 0"/>
      <xacro:wheel_geometry />
      <material name="red"/>
    </visual>
</link>
<joint name="joint_left_wheel" type="continuous">
    <origin rpy="0 0 0" xyz="0.25 -0.15 0"/>
    <child link="link_left_wheel"/>
    <parent link="link_chassis"/>
    <axis rpy="0 0 0" xyz="0 1 0"/>
</joint>

<!-- Additional wheels, adjusted positions and naming -->
<!-- Additional Right Wheel -->
<link name="link_additional_right_wheel">
    <xacro:default_inertial mass="0.2"/>
    <collision name="link_additional_right_wheel_collision">
      <origin rpy="0 1.5707 1.5707" xyz="0 0 0"/>
      <xacro:wheel_geometry />
    </collision>
    <visual name="link_additional_right_wheel_visual">
      <origin rpy="0 1.5707 1.5707" xyz="0 0 0"/>
      <xacro:wheel_geometry />
      <material name="red"/>
    </visual>
</link>
<joint name="joint_additional_right_wheel" type="continuous">
    <origin rpy="0 0 0" xyz="-0.25 0.15 0"/>
    <child link="link_additional_right_wheel"/>
    <parent link="link_chassis"/>
    <axis rpy="0 0 0" xyz="0 1 0"/>
</joint>

<!-- Additional Left Wheel -->
<link name="link_additional_left_wheel">
    <xacro:default_inertial mass="0.2"/>
    <collision name="link_additional_left_wheel_collision">
      <origin rpy="0 1.5707 1.5707" xyz="0 0 0"/>
      <xacro:wheel_geometry />
    </collision>
    <visual name="link_additional_left_wheel_visual">
      <origin rpy="0 1.5707 1.5707" xyz="0 0 0"/>
      <xacro:wheel_geometry />
      <material name="red"/>
    </visual>
</link>
<joint name="joint_additional_left_wheel" type="continuous">
    <origin rpy="0 0 0" xyz="-0.25 -0.15 0"/>
    <child link="link_additional_left_wheel"/>
    <parent link="link_chassis"/>
    <axis rpy="0 0 0" xyz="0 1 0"/>
</joint>

</robot>
