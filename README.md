# FALCON: Fixed-Wing Aerial Lifting and Carying of Objects inspired by Nature 
Repository with all the code that was written in the FALCON project.

## Installation
TODO: installation procedure

## Description of all the apps
The following list contains short descriptions of the apps in this repo. The command line examples assume that the flight controller is attached to the port: ``` /dev/ttyUSB0 ``` and the Arduino to the port ```/dev/ttyACM0 ```. 

### Flight Logger
Runs on the onboard raspberry pi. It reads data from the flight controller and stores it in a csv file.

Command line arguments:
```
./build/apps/app_flight_logger/flight_logger serial:///dev/ttyUSB0
```

### Grasper
Runs on the onbaord raspberry pi. This code starts the fixed-wing mission that was uploaded to the flight controller prior (please verify that the mission makes sense and the setpoints in the code work with the mission). When the plane enters the grasp-zone the offboard controller is activated which sends commands from an offline computed trajectory. This should result in a grasping sequence. After the sequence is finished, the mission continues and the plane lands.

Command line arguments:
```
./build/apps/app_grasper/grasper serial:///dev/ttyUSB0 /dev/ttyACM0
```

### Gripper mocap test
Runs on the onbaord raspberry pi. This app was designed to test the gripper. It uses the vicon-datastream kit to get position information from the vicon system.

Command line arguments:
```
./build/apps/app_gripper_mocap_test/gripper_mocap_test /dev/ttyACM0
```
### Gripper test
Runs on the onboard raspberry pi. With this app, one can check if the gripper works. Just type in angle setpoints into the console and the gripper should go into that position.

Command line arguments:
```
./build/apps/app_gripper_test/gripper_test /dev/ttyACM0
```

### Mission [might be removed]
Runs on the onboard raspberry pi. This app was used to test the deployment of the custom controller. It works similar to Grasper, but does not activate a grasping sequence. It recognizes that the plane is in the grasping zone but continues to normally fly. 

Command line arguments:
```
./build/apps/app_mission/mission /dev/ttyACM0
```

### MPC plane [might be removed]
This is a possible MPC implentation, that doesnt work because the optimizer converges too slow. check Grasper for a working implemetation.

### Multithread test
This app tested the multithreading needed for an MPC where optimization and control processes run simultaniousely.

### Trajectory generator (traj_gen)
This app runs on a powerful ground station computer. This app computes optimal trajectories for the plane to fly for different initial conditions. We run this before we fly, and then copy the file "opt_ctrl_log.csv" to the grasper app. It will then read out the optimal control values calculated here when it starts its offboard control loop.

## Description of all helper apps

### arduino_code/gripper
This arduino code is uploaded to the onboard arduino nano. It listens to Serial events on the USB Port of the arduino which is attached to the Raspberry Pi. These serial messages contain angle-setpoints for the gripper arm servo. The arduino translates it into a pwm signal which is sent to the servo.

### sysID
This python script reads in log files from the flight logger apps and determines lift and drag coefficients of the plane.

### thrust-throttle
This python script uses force measurments from a test bench to determine the propellers throttle-thrust relation.

### visualize MPC
This python script visualizes the MPC libs trajectories visually.

### waypoints

This python script calculates waypoints in a straight line. We can enter the object's coordinates, a desired grasp-length and direction and it gives us the coordinate were we enter the grasp and where we exit it.

## Simulation
TODO: write how to set up simulation.