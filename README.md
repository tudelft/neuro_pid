# SpikingPIDRos

These steps assume you have already installed ROS Melodic and sourced your environment on a machine running Linux Ubuntu 18.04 OS.
If that is not the case, the instructions on https://github.com/ethz-asl/rotors_simulator for installing ROS also work for ROS Melodic, by replacing indigo with melodic.
It is also expected that a version of Anaconda/Miniconda is installed on the machine.

Steps:

- create a conda environment and install necessary python packages

```
conda create --name npid python=3.6
conda activate npid
```
TODO: CREATE TXT WITH PYTHON PACKAGES AND ADD

- Follow the instructions for installing RotorS in a new workspace and the RotorS simulator (https://github.com/ethz-asl/rotors_simulator)
Remember to exchange the ROS version for melodic everywhere.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox ros-melodic-mavros python-rosdep python-rosinstall python-rosinstall-generator build-essential

sudo rosdep init
rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

export ROS_PYTHON_VERSION=3
```

Following might be necessary:
```
pip install empy

```

TODO: PROBABLY SOME PYTHON THINGS HERE THAT NEED TO BE DONE AS WELL

- clone this repository and copy the files in the folder /src/ to the /src/ folder in your catkin workspace:
```
git clone https://github.com/sstroobants/SpikingPIDRos.git
cp SpikingPIDRos/src/ ~/catkin_ws/src/
```

- Make python files in spiking_pid executable:
```
cd ~/catkin_ws/src/spiking_pid/scripts
chmod +x gazebo_position_controller.py optitrack_node.py waypoint_node.py inav_height_controller_cyberzoo_loihi.py inav_position_controller_cyberzoo.py run_simulation_tests.py
```

- build all ROS packages
```
catkin build
SOURCE DEVEL
```



# Simulation

## Changing simulation parameters
- The update frequency of the simulation can be set in 
~/catkin_ws/src/rotors_simulator/rotors_gazebo/worlds/basic.world by changing the max_step_size and real_time_factor. By changing the real_time_update_rate to 0, the simulation will run as fast as possible. 

- the precision of the N-PID for simulation can be set by changing the precision parameter in the controller_gazebo_height.launch launch-file.
Here also other parameters considering the quadrotor can be changed, such as the PID gains.

## Run simulation
- Start a roscore
```
roscore
```

- Start Gazebo and spawn a hummingbird quadrotor
```
roslaunch spiking_pid start_simulation.launch
```

- Start the controller node, first resetting the quadrotor and then going to the setpoint published to /waypoints
```
roslaunch spiking_pid controller_gazebo_height.launch
```


## Multiple runs
To do multiple runs for different heights, the run_simulation_tests.py file can be used. In this file, the amount of runs and a list of heights can be given. 

# License

The RotorS Simulator and the MSF framework are both licensed under the Apache 2.0 license. 
In the RotorS Simulator, only two minor changes have been made:
The rate in basic.world has been changed and a generic sensor for the hummingbird has been added.
The MSF Framework has not been changed.