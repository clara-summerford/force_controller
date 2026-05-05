# force_controller
This is a twist-based force controller. It commands an end-effector twist to achieve a target force based on the force feedback in the desired dimension. This controls the force exerted at the end effector of the robot arm. It leverages a simple PID controller setup for control and a wrench filter to smooth the wrench data.

The responsiveness of the behavior can be tweaked by the following parameters: The p, i and d gains determine the responsiveness in the desired Cartesian axis. The higher these values, the faster the robot moves in response to either the sensed contact forces or the target wrench. The proportional term, p, adjusts the control action based on the current error, the integral term, i, integrates past errors to eliminate steady-state errors, and the derivative term, d, anticipates future errors based on the rate of change of the error. Proportional gains (p) are normally sufficient and you'll probably not need an i or d gain for most use cases. With a little bit of tweaking, though, they sometimes help to reduce oscillations in stiff contacts. 

This controller works with both single and multi-dimensional force control problems. 
 
#### Tips for tuning: 
Tune the gains to control the speed or time it takes to reach the desired force. 

p: Start with a small value (e.g., 0.1) and gradually increase it until the system starts to respond well without excessive oscillations.

i : Start with a very small value (e.g., 0.001) to prevent integral windup and gradually increase it until the steady-state error is reduced. Be cautious with high integral gains, as they can lead to overshooting and instability.

d: Start with a small value (e.g., 0.01) and gradually increase it to improve system response and damping. However, be cautious not to set it too high, as it can introduce noise and lead to instability.

## Installation 
### Requirements
- A force torque sensor
- A twist controller (MoveIt Servo is preferred). Checkout the twist_controller branch if your controller requires Twist messages instead of TwistStamped messages. 
 
### Standard dependencies
This package is based on [ROS2](https://docs.ros.org/). Currently based on ROS2 humble release.

### Download the pacakge
```
  export COLCON_WS=~/workspace/force_controller_ws
  mkdir -p $COLCON_WS/src
  cd $COLCON_WS
  git clone https://github.com/UTNuclearRobotics/ros2_force_controller.git src/force_controller_ws
  ```

#### Install dependencies, compile and source the package
We use [colcon](https://github.com/machines-in-motion/machines-in-motion.github.io/wiki/use_colcon)
to build this package:
```
sudo apt-get install ros-humble-moveit-servo
rosdep install --ignore-src --from-paths src -y -r
cd $COLCON_WS
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Usage
The launch file launches the force_controller node with the following some default configuration eg. it's defaulted to publish on the servo node. Change the values in the config file to fit your specific setup. For now, only a single dimension is force-controlled. Set the desired target force as a parameter in the config file. 

To launch the node, run:
```
ros2 launch force_controller twist_force_controller.launch.py
```

## Safety
- This package uses maximum twist and wrench values to avoid damaging the robot or the task. 
- It only publishes twists if force-torque sensor data available on initialization. 
- If the force-torque data comes close to the max, or close to zero (stops publishing), the controller publishes 0 twists. 
 
## Sensor data filtering 
Force - torque sensor data is typically noisy due to a variety of factors, such as sensor vibrations, electrical interference, and mechanical vibrations. Using a filter can help to remove this noise and provide a smoother signal that is easier to control. The following filters are provided: 
- Deadband (set deadband_threshold to 0 to remove deadband)
- Exponential Moving Average Filter
- 2nd Order Butterworth Filter
- Kalman Filter 

Explanations/tips are provided in the configuration file. Set the controller parameters accordingly based on desired filtering behavior.  Carefully tune these values to avoid introducing (phase) lag or smoothing out crucial information.

## Additional Insights
- When loaded, the EE will move with a low twist along the desired force control dimension, until it starts to feel a force above 1 N. This will trigger the controller sequence of adjusting the twist relative to the desired force and in response to the measured ft force. 

 - The are three ways to set the target force:
   1. Pass a value in the config file 
   2. Programmatically call the setTargetForce() to update the target_force_ variable
   3. Publish the desired force on a topic. This has to be a double, and should be in the desired dimension already set in the config file. 

    The config file value takes precedence and is set on the initialization of the node. After that the value can be changed. In this case, another node can publish a series of setpoints for the controller to achieve.

    You have the option to control one dimension or all six dimensions; the default is single dimension force control. Use this if you do not care/ do not wish to actively control the other wrench dimensions. Eg. Instead of 0 N force control in the x direction, just ignore it to avoid actively trying to achieve 0 N in that direction. Set/ensure that the `control_wrench` parameter is `False`. 
    
    Use multi-dimension force control to apply a wrench through active control of all dimensions. For this, set the `control_wrench` parameter to `True` and set the `target_wrench` parameter. You can then set the wrench according to the three ways listed above. You can leave `target_force` blank in this case. 
    
    Note: Use only one 1 control scheme at a time.

### Main Equation 
This is framed as a quasi-static problem, thus ignoring acceleration. To maintain equilibrium, the equation:

F_target - kx - bv = 0 

where, F_target is the desired force, k is the stiffness, x is the displacement, b is the damping, and v is the velocity, helps to expresses the target force required to maintain force equilibrium, with the restoring force (kx) acting towards equilibrium and the damping force (bv) opposing velocity. 
 
### Feedforward term: 
Helps to eliminate a significant portion of setpoint and disturbance errors before reaching the controller. It anticipates the force required at each 'waypoint' based on the desired trajectory, aiding in pre-compensating for the expected force thereby reducing settling time.

This can be applied to constant setpoints, but is generally advised to use the ff for a changing setpoint. Adjust the config file params accordingly. 

Note: While dropping the feedforward term simplifies the control scheme, it might not be the optimal choice if accurate force tracking, increased overshoot and settling time, fast response times, and energy efficiency are crucial considerations. Please tune the variables according to your system dynamics and conditions for optimal performance. 
