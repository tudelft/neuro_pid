import numpy as np
# import numpy.linalg as npl

def to_euler_angles(q):
    w, x, y, z = q[0],q[1],q[2],q[3]

    # // roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # // pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    # // yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return np.array([pitch, roll, yaw]).T


def calculate_allocation_matrix(k, b, l, orientation='x'):
    # x-oriented quadrotor:
    if orientation == 'x':
        Mw = np.array([[-l * k / np.sqrt(2), -l * k / np.sqrt(2), l * k / np.sqrt(2), l * k / np.sqrt(2)],
                        [l * k / np.sqrt(2), -l * k / np.sqrt(2), l * k / np.sqrt(2), -l * k / np.sqrt(2)],
                        [-b                 , b                 , b                , -b ],
                        [k                  , k                 , k                 , k]])

    elif orientation == '+':
    # +-oriented quadrotor:
        Mw = np.array([[0, l * k, 0, -l * k],
            [-l * k, 0, l * k, 0],
            [b  , -b   , b , -b ],
            [k   , k   , k  , k]])
    return Mw


def thrust_to_rpm(thrust, k_f):
    rpm = np.sqrt(thrust / k_f)
    return rpm


def rpm_to_rads(rpm):
    return (rpm * 2 * np.pi) / 60

def omega_to_forces(omega, m_w):
    omegasq = omega**2
    Tbar = m_w @ omegasq
    T = Tbar[3]
    tau = Tbar[0:-1]
    return T, tau

def forces_to_omega(forces, m_w_inv, min_omega=0, max_omega=838):
    w_sq = m_w_inv @ forces
    w1sq, w2sq, w3sq, w4sq = w_sq
    w1 = min(np.sqrt(max(w1sq, min_omega)), max_omega)
    w2 = min(np.sqrt(max(w2sq, min_omega)), max_omega)
    w3 = min(np.sqrt(max(w3sq, min_omega)), max_omega)
    w4 = min(np.sqrt(max(w4sq, min_omega)), max_omega)
    omega = [w1, w2, w3, w4]
    return omega