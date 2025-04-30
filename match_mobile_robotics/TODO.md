# TODO
## move_group:
In mur_moveit_config/launch/move_group.launch launched by start_moveit_620.launch:
- **false robot description is used** for planning_context.launch etc. has to use mur620.srdf.xacro instead of mur.srdf.xacro (only for mur620, for mur100 ok)

## disable state pub of ur!
- right now just disabled in sub package
- see match_mobile_robotics/ur/ur_ros_driver/ur_robot_driver/launch/ur_common.launch

## HW/Sim identic

### Twist-Controller:
tool0_controller tf is now remapped to /tf_old
- -> change name from UR10_r/base -> UR10_r/tool0_controller to /mur620/UR10_r/... and publish to /tf


### Set Caster_joints
- to 0 or fixed
- otherwise always warning from move_group

### SIM:
prob. have to set in mir_launch_sim:

    ```   
    <remap to="cmd_vel" from="mobile_base_controller/cmd_vel"/>
    <remap from="odom" to="mobile_base_controller/odom"/>
    ```
