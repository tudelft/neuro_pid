#!/usr/bin/env python
import os
import rospy
import roslaunch
from pathlib import Path
from std_srvs.srv import Empty


if __name__=="__main__":
    rospy.init_node('run_simulations', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    # Create service handle to unpause Gazebo physics
    rospy.wait_for_service("/gazebo/pause_physics")
    pauser = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
    home = str(Path.home())
    
    # For all desired heights
    for height in [1, 1.5, 2, 2.5, 3]:
        os.mkdir(os.path.join(home, '.ros', f'{height}m'))
        # For each simulation run
        for i in range(1,4):
            cli_args = ["/home/sstroobants/catkin_ws/src/spiking_pid/launch/controller_gazebo_height.launch",f'filename:={height}m/{height}m{i}', f'waypoint:={height}']
            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
            launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            launch.start()
            rospy.loginfo("started")

            rospy.sleep(25)
            paused = pauser()
            # 3 seconds later
            launch.shutdown()