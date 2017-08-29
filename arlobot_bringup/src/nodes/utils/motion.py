import math

def uni2diff(v, w, L, R):
    """
    Convert unicyle to differential
    :param v: Linear velocity
    :param w: Angular velocity
    :param L: Wheel base (or track) width
    :param R: Wheel radius
    :return: left/right velocity
    """
    #print("U2D - {:6.3f} {:6.3f} {:6.3f} {:6.3f}".format(v, w, L, R))
    _2_v = 2 * v
    _w_L = w * L
    _2_R = 2 * R

    try:
        l_v = (_2_v - _w_L) / _2_R
        r_v = (_2_v + _w_L) / _2_R
    except ZeroDivisionError:
        l_v, r_v = 0, 0

    return l_v, r_v


def diff2uni(l_v, r_v, L, R):
    """
    Convert differential to unicycle
    :param l_v: left velocity
    :param r_v: righ velocity
    :param L: Wheel base (or track) wdith
    :param R: Wheel radius
    :return: Linear/Angular velocity
    """
    #print("D2U - {:6.3f} {:6.3f} {:6.3f} {:6.3f}".format(l_v, r_v, L, R))

    v = (l_v + r_v) * (R / 2)
    try:
        w = (r_v - l_v) * (R / L)
    except ZeroDivisionError:
        v, w = 0, 0

    return v, w


def ensure_w(v, w, track_width, wheel_radius, ang_wheel_max):
    """
    Ensure specified angular velocity can be met by adjusting linear velocity
    :param v: Linear velocity
    :param w: Angular velocity
    :param track_width: Wheel base width
    :param wheel_radius: Wheel radius
    :param ang_wheel_max: Maximum allowed angular velocity
    :return: Linear/Angular velocity
    """
    l_v_d, r_v_d = uni2diff(v, w, track_width, wheel_radius)

    max_rl_v = max(l_v_d, r_v_d)
    min_rl_v = min(l_v_d, r_v_d)

    l_v = 0
    r_v = 0

    #print("{:6.3f} {:6.3f} {:6.3f} {:6.3f} {:6.3f}".format(l_v_d, r_v_d, track_width, wheel_radius, ang_wheel_max))

    # Only adjust if v and w are non-zero
    if v != 0.0 and w != 0.0:
        if max_rl_v > ang_wheel_max:
            r_v = r_v_d - (max_rl_v - ang_wheel_max)
            l_v = l_v_d - (max_rl_v - ang_wheel_max)
        elif min_rl_v < -ang_wheel_max:
            r_v = r_v_d - (min_rl_v + ang_wheel_max)
            l_v = l_v_d - (min_rl_v + ang_wheel_max)
        else:
            l_v = l_v_d
            r_v = r_v_d

        return diff2uni(l_v, r_v, track_width, wheel_radius)
    else:
        return v, w


def x_dot(left, right, radius, theta):
    return (radius * (left + right) * math.cos(theta) )/ 2.0

def y_dot(left, right, radius, theta):
    return ( radius * (left + right) * math.sin(theta) ) / 2.0

def theta_dot(left, right, radius, track_width):
    return radius * (right - left) / track_width

def uni_max(max_wheel_angular_vel, track_width, wheel_radius):
    """
    Computes the maximum possible linear and angular velocities given a maximum wheel velocity
    Note: Assume forward and backward wheel velocities are the same

    :param max_wheel_angular_vel: maximum wheel angular velocity (typically derived from motor RPM) in radians/second
    :param track_width: distance between left and right wheels (in meters)
    :param wheel_radius: (in meters)
    :return:
    """
    max_lin_vel, _ = diff2uni(max_wheel_angular_vel,
                              max_wheel_angular_vel,
                              track_width,
                              wheel_radius)

    _, max_ang_vel = diff2uni(-max_wheel_angular_vel,
                              max_wheel_angular_vel,
                              track_width,
                              wheel_radius)

    return max_lin_vel, max_ang_vel

def diff_max(max_wheel_angular_vel, track_width, wheel_radius):
    lin, ang = uni_max(max_wheel_angular_vel, track_width, wheel_radius)

    lin_left, lin_right = uni2diff(lin, 0, track_width, wheel_radius)
    ang_left, ang_right = uni2diff(0, -ang, track_width, wheel_radius)

    return max(lin_left, ang_left), max(lin_right, ang_right)