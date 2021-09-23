import numpy as np
import rospy

from body_equations import *
from src.spiking_pid_numpy import SpikingPIDSplitted
from src.encode_numpy import position_coding

class PID:
    def __init__(self, Kp, Ki, Kd, verbose=False, integral_max=None):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.integral = 0
        self.err_prev = 0
        self.verbose = verbose
        self.deriv_prev = 0
        self.integral_max=integral_max

    def calculate_output(self, commanded, measured, dt):
        err = commanded - measured

        prop = self.Kp * err

        self.integral = 0.90 * self.integral + err * dt
        integ = self.Ki * self.integral
        if self.integral_max is not None:
            # print(min(self.integral_max, integ))
            self.integral = max(min(self.integral_max / self.Ki, self.integral), -self.integral_max / self.Ki)
        
        # deriv = (err - self.err_prev) / dt
        deriv = ((err - self.err_prev) / dt) * 0.1 + self.deriv_prev * 0.9
        # deriv = max(min(0.5, deriv), -0.5)
        self.deriv_prev = deriv
        deriv = self.Kd * deriv
        self.err_prev = err
        if self.verbose:
            rospy.loginfo(('parts: ', prop, integ, deriv))
        output = prop + integ + deriv
        return output


class AttitudePID:
    g = 9.8

    def __init__(self, gains, m, l, k_f, k_d, with_delay=False, spiking_precision=63, orientation='x', use_loihi_weights=False):
        self.gains = gains
        self.m = m
        self.l = l
        self.k_f = k_f
        self.k_d = k_d
        self.m_w = calculate_allocation_matrix(k_f, k_d, l, orientation=orientation)
        self.m_w_inv = np.linalg.inv(self.m_w)
        self.spiking_precision = spiking_precision
        self.alt_d_prev = None
        self.pitch_d_prev = None
        self.roll_d_prev = None

        self.altitude_pid = SpikingPIDSplitted(gains['z']['Kp'], 
                                gains['z']['Ki'],
                                gains['z']['Kd'], 
                                np.linspace(4, 0, spiking_precision),
                                np.linspace(2, -2, spiking_precision),
                                np.linspace(0.5, -0.5, spiking_precision), 
                                np.linspace(1.25, -1.25, spiking_precision),
                                0.01,
                                with_delay=with_delay, 
                                use_loihi_weights=use_loihi_weights)

        self.pitch_pid = PID(gains['pitch']['Kp'], 
                                gains['pitch']['Ki'],
                                gains['pitch']['Kd'],
                                verbose=False)

        self.roll_pid = PID(gains['roll']['Kp'], 
                                gains['roll']['Ki'],
                                gains['roll']['Kd'])

        self.yaw_pid = PID(gains['yaw']['Kp'], gains['yaw']['Ki'], gains['yaw']['Kd'])


    def calculate_rotor_speeds(self, attitude_d, attitude, altitude_d, altitude, vel, ang_vel, dt):
        # rospy.loginfo([attitude_d, attitude, altitude_d, altitude, ang_vel, dt])
        pitch_d, roll_d, yaw_d = attitude_d
        pitch, roll, yaw = attitude

        altitude_enc = position_coding(
            altitude, self.spiking_precision, min_input=0, max_input=4
        )
        altitude_d_enc = position_coding(
            altitude_d, self.spiking_precision, min_input=0, max_input=4
        )

        if self.alt_d_prev == None:
            self.alt_d_prev = altitude_d
        altitude_deriv_error = ((altitude_d - self.alt_d_prev) / dt) - vel.z
        self.alt_d_prev = altitude_d
        # rospy.loginfo(altitude_deriv_error)

        altitude_deriv_error_enc = position_coding(
            altitude_deriv_error, self.spiking_precision, min_input=-0.5, max_input=0.5
        )

        self.altitude_d_enc = altitude_d_enc
        self.altitude_enc = altitude_enc
        self.altitude_deriv_error_enc = altitude_deriv_error_enc
        thrust_d = self.m * self.g + 0.45 + self.altitude_pid.calculate_output(altitude_d_enc, altitude_enc, altitude_deriv_error_enc) 

        pitch_t_d = self.pitch_pid.calculate_output(pitch_d, pitch, dt)

        roll_t_d = self.roll_pid.calculate_output(roll_d, roll, dt)

        yaw_d = self.yaw_pid.calculate_output(yaw_d, yaw, dt)

        # rospy.loginfo([roll_t_d, pitch_t_d, yaw_d, thrust_d])
        # omega = forces_to_omega(np.array([roll_t_d, pitch_t_d, yaw_d, thrust_d]), self.m_w_inv)
        return np.array([roll_t_d, pitch_t_d, yaw_d, thrust_d])
        # return omega

class PositionPID:
    def __init__(self, gains, 
                        m, 
                        l, 
                        k_f, 
                        k_d, 
                        max_angle=0.52, 
                        with_delay=False, 
                        spiking_precision=63, 
                        orientation='x', 
                        use_loihi_weights=False):
        self.m = m
        self.l = l
        self.k_f = k_f
        self.k_d = k_d
        self.max_angle = max_angle

        self.x_pid = PID(gains['x']['Kp'], gains['x']['Ki'], gains['x']['Kd'], verbose=False)
        self.y_pid = PID(gains['y']['Kp'], gains['y']['Ki'], gains['y']['Kd'])

        self.att_pid = AttitudePID(gains, 
                                    m, 
                                    l, 
                                    k_f, 
                                    k_d,
                                    with_delay=with_delay, 
                                    spiking_precision=spiking_precision, 
                                    orientation=orientation,
                                    use_loihi_weights=use_loihi_weights)

    def calculate_attitude_commands(self, pos_d, pos, yaw, dt):
        x_d, y_d = pos_d
        x, y = pos
        # rotate desired (x, y)-position to body coordinates
        x_d_body, y_d_body =  np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]]) @ np.array([x - x_d, y - y_d]).T
        # rospy.loginfo([x_d_body, y_d_body])
        # calculate desired angles and saturate with maximum desired angles
        pitch_d = self.x_pid.calculate_output(x_d_body, 0, dt)
        pitch_d = min(max(pitch_d, -self.max_angle), self.max_angle)
        roll_d = self.y_pid.calculate_output(y_d_body, 0, dt)
        roll_d = min(max(roll_d, -self.max_angle), self.max_angle)
        # rospy.loginfo(f'{yaw, x,  x_d_body, -pitch_d, y, y_d_body, roll_d}')
        # yaw is set to zero for now
        # return [0, 0, 0]
        return [-pitch_d, roll_d, 0]

    def calculate_rotor_commands(self, pos_d, pos, att, vel, ang_vel, dt):
        x, y, z = pos[0], pos[1], pos[2]
        x_d, y_d, z_d = pos_d[0], pos_d[1], pos_d[2]
        yaw = att[2]
        att_d = self.calculate_attitude_commands([x_d, y_d], [x, y], yaw, dt)
        # rospy.loginfo(att_d)
        omega = self.att_pid.calculate_rotor_speeds(att_d, att, z_d, z, vel, ang_vel, dt)
        return omega

