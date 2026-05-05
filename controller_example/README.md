# Running the force controller example
The force controller is a twist-based controller that requires a twist controller or MoveIt Servo. This minimal examples leverages a standalone node that publishes a force along the z-axis as an input to the force controller in lieu of an actual force-torque sensor. Please refer to the [README](../README.md) in the main directory for more details. 

Below are two ways to test the controller. 
1.  Using the Panda robot with RViz2
2.  Using the Kinova robot with MoveIt Studio

Both assume that the force controller workspace has been built and sourced. If you haven't already done so already, make sure to complete the installation steps in the [README](../README.md). 

## Panda Robot RViz2 Instructions
1. Install the Panda MoveIt config package and MoveIt Servo
```
sudo apt install ros-humble-moveit-resources-panda-moveit-config
sudo apt install ros-humble-moveit-servo
```

2. Launch the Servo node. This will bring up the Panda robot using the panda-moveit-config package.  
```
ros2 launch moveit_servo servo_example.launch.py
```

* If you have issues with any of the steps so far, follow the instructions on [this page](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html#getting-started)

* Optionally edit the parameters in the config/fake_wrench_pub.yaml file

3. Open a new terminal and launch the fake_wrench_publisher 
```
cd ~/workspace/force_controller_ws/ && colcon build && source install/setup.bash
ros2 launch force_controller fake_wrench_pub.launch.py
```

4. Open a new terminal, and launch the force controller. 
```
cd ~/workspace/force_controller_ws/ && source install/setup.bash
ros2 launch force_controller panda_force_controller.launch.py
```
The Panda starts to move slowly upward and downward while trying to maintain the force setpoint in response to the published fake wrench.

* Optionally open rqt to visualize the wrench data
```
Launch rqt, navigate to Plugins -> Visualization -> Plot
Type /wrench_data/wrench/force/z into the Topic entry text bar 
```
* Optionally echo the servo_node topic to see the published twist messages
```
ros2 topic echo /servo_node/delta_twist_cmds
``` 

## Kinova Robot MoveIt Pro Instructions
This assumes that MoveIt Pro has been installed and properly configured. If you haven't already done so already, make sure to complete the installation steps in the [README](../../../../README.md). 

1. After building Docker and the user workspace, launch MoveIt Pro
```
cd ~/picknik_ut_moveit_studio_ws && docker compose up
```

2. Bring up the UI in a web browser
```
localhost/
```

3. Under the Objective Builder tab, select Force Controller Minimal Example

Optional: If you wish to adjust the configuration parameters, select the `Edit` button in the top right corner, and 
change the parameters in each behavior accordingly. 

4. Click `Run`. This will actuate the Kinova arm backwards incrementally for 10 seconds based on the published fake wrenches. 

Optional: With DDS enabled (see [README](../../../../README.md)), you can echo the `/studio_ui/move_end_effector` topic or `\wrench_data` topic to visualize the published data

### Testing Tips
* Explore how changing the 'force_setpoint' and 'dimension' parameters in panda_controller_params.yaml impact the robot behavior. This can be used in conjuction with the parameters in the config/fake_wrench_pub.yaml file. 

* Only use the filter parameters if you have read and understand their impact on wrench data. By default, they are all set to false for this minimal example.
* Optionally adjust the EE velocity by changing the damping parameter. However, this should be done with an understanding of how it affects the quasi-static model. 
* Optionally adjust the P, I and D (and the ff_gain) parameters to investigate how they affect they controller performance. 