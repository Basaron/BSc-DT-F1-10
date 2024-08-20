# Digital Twin for the F1Tenth Race Car
This is a digital twin for the F1Tenth car project.  With simulations in INTO-CPS

This project is a Bachelor project in 

A demo of the 
https://youtu.be/-DEOV0oWJdw


## Prerequisites and Installation
- Python 3.7 or higher
- Robot Operating System (ROS)
- RabbitMQ
- The INTO-CPS application. 

In order to use the code in this repository, follow the steps:
1. Copy the ```f1tenth_simulator_modified``` folder into your catkin workspace and build the workspace
2. Copy the ```ros_nodes``` into your catkin workspace e.g. the ```f1tenth_simulator_modified``` folder
3. Then copy the ```f1tenth_intoCPS``` into your INTO-CPS project folder

## Usage
For running the digital twin, five terminals are needed. 


Terminal 1: Launch the simulator for the F1Tenth
```bash
…\catkin_ws $ roslaunch f1tenth_simulator simulator.launch
```

Terminal 2: Initialize the LIDAR sensor
```bash
…\ros_nodes $ python3 lidar_sensor.py
```

Terminal 3: Initialize the script that processes the LIDAR data
```bash
…\ros_nodes $ python3 lidar_data_processer.py
```

Terminal 4: Initialize the odometry sensor
```bash
…\ros_nodes $ python3 odom_sensor.py
```

Terminal 5: Run the controller
```bash
…\ros_nodes $ python3 moter_actuator.py
```

To run the simulation, go into the INTO-CPS application and open the f1tenth_intoCPS project. In the application, it is possible to configure the project setup e.g. determine simulation time or step size. 

### Authors and Contributors 
Bastian Aron Kramer

GitHub: https://github.com/Basaron


Emil Chao Hu

GitHub: https://github.com/emilhu20


Malthe Tøttrup Birkebæk

GitHub: https://github.com/MaltheT/MaltheT
