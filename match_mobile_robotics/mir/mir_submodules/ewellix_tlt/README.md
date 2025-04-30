# The ewellix TLT ROS package

This ROS package allows you to control or simulate an Ewellix TLT lifting column.
It is compatible with the Kinetic and melodic versions.

## Dependencies
```bash
sudo apt-get install -y libserial-dev
```
## Use in simulation

```bash
roslaunch ewellix_tlt demo_simu_ewellix.launch
```
Once the program is launched you can move the column joint via the joint_state_publisher_gui interface.

## Use in real conditions
Connect the column to your computer and modify the launcher to set the port. By default it is ttyUSB0.

Then use:

```bash
roslaunch ewellix_tlt component_ewellix.launch
```
Once the program is launched, you can control the size of the column by posting on the topic: /ewellix_tlt/size 
Value between 0.0 and 0.5.