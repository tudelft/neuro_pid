#!/usr/bin/env python
import numpy as np
import rospy
import yaml
import rospkg
import os

from geometry_msgs.msg import Point, Quaternion, Vector3
from mav_msgs.msg import Actuators, TorqueThrust
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from sensor_fusion_comm.srv import InitScale
from body_equations import *
from neuromorphic_altitude_pid import *
from src.adder_numpy import create_csv_from_adder


def odo_callback(data):
    global pos, att, t, ang_vel, vel
    # t_new = rospy.get_time()
    # # rospy.loginfo(rospy.get_time() - t)
    # t = t_new
    pos = data.pose.pose.position
    att = data.pose.pose.orientation
    vel = data.twist.twist.linear
    ang_vel = data.twist.twist.angular
    

def waypoint_callback(data):
    global waypoint
    waypoint = data

def listener():
    global pos, att, motor_command, pid, pub, att_gt, t, ang_vel, waypoint, vel, bag
    last_time = rospy.get_time()
    t = last_time
    rospy.Subscriber("/odometry", Odometry, odo_callback, queue_size=5, buff_size=2**24, tcp_nodelay=True)
    rospy.Subscriber("/waypoints", Point, waypoint_callback, queue_size=1)

    rate = rospy.Rate(200)
    rospy.sleep(3)
    att_last = None
    started = False
    while not rospy.is_shutdown():
        if (not started) and (waypoint.x is not None) and (att != att_last):
            started = True
            sim_started.data = True
            status_pub.publish(sim_started)
            rospy.sleep(1)

        if started:
            rtime = rospy.get_time()
            dt = rtime - last_time
            dt = max(dt, 0.002)
            
            last_time = rtime
            q = att
            eulers = to_euler_angles([q.w, q.x, q.y, q.z])
            pos_d = [waypoint.x, waypoint.y, waypoint.z]
            forces = pid.calculate_rotor_commands(pos_d, [pos.x, pos.y, pos.z], eulers, vel, ang_vel, dt)
            
            rpm = forces_to_omega(forces, pid.att_pid.m_w_inv)
            motor_command.angular_velocities = rpm
            pub.publish(motor_command)
            att_last = att
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    global pos, motor_command, att, pid, pub, ang_vel, waypoint, bag
    
    rospack = rospkg.RosPack()
    pos = Point()
    motor_command = Actuators()
    att = Quaternion()
    ang_vel = Vector3()
    vel = Vector3()
    waypoint = Point()
    waypoint.x, waypoint.y, waypoint.z = None, None, None 
    #initial waypoint is None, so it will only start after receiving the first waypoint

    gains_file = rospy.get_param("spiking_pid/gains")
    orientation = rospy.get_param("spiking_pid/orientation")
    log_filename = rospy.get_param("ros_logger/filename")
    spiking_precision = rospy.get_param("spiking_pid/precision")
    control_range = rospy.get_param("spiking_pid/control_range")

    with open(os.path.join(rospack.get_path('spiking_pid'), gains_file)) as f:
        gains = yaml.load(f, Loader=yaml.FullLoader)

    M = 0.68
    G = 9.8
    K_F = 8.54858e-06
    L = 0.17
    K_D = K_F * 0.016

    pid = PositionPID(gains, M, 
                            L, 
                            K_F, 
                            K_D, 
                            with_delay=True, 
                            spiking_precision=spiking_precision, 
                            orientation=orientation,
                            use_loihi_weights=True)

    create_csv_from_adder('error', pid.att_pid.altitude_pid.error_sub)
    create_csv_from_adder('int', pid.att_pid.altitude_pid.int_adder)
    create_csv_from_adder('control', pid.att_pid.altitude_pid.control_adder)


    rospy.init_node('gazebo_position_controller')

    status_pub  = rospy.Publisher('/simulation_started', Bool, queue_size=1)
    sim_started = Bool()
    sim_started.data = False

    pub = rospy.Publisher('/command/motor_speed', Actuators, queue_size=1)

    # Create service handle to unpause Gazebo physics
    rospy.wait_for_service("/gazebo/unpause_physics")
    unpauser = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    unpaused = unpauser()

    rospy.sleep(3)
    # Send zero motor speed command to gazebo
    motor_command = Actuators()
    motor_command.angular_velocities = [0, 0, 0, 0]
    pub.publish(motor_command)

    # Create service handle to reset the environment
    rospy.wait_for_service("/gazebo/reset_world")
    reset = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    is_reset = reset()

    rospy.sleep(0.5)

    rospy.wait_for_service("hummingbird/msf/pose_sensor/initialize_msf_scale")
    init_srv = InitScale()
    msf_init = rospy.ServiceProxy("hummingbird/msf/pose_sensor/initialize_msf_scale", InitScale)
    msf_started = msf_init(1)
    rospy.sleep(1)

    try:
        listener()
    except rospy.ROSInterruptException:
        pass