import numpy as np
from matplotlib import pyplot as plt
from matplotlib import cm
from math import sin, cos, pi
from scipy import signal

im_sensor_size = [4e-3, 3e-3]


def calcImPlaneZ(fov):
    """
    Assume sensor size is 3 x 4 mm
    :param fov: fov of camera
    :return: depth of image plane
    """
    return im_sensor_size[0] / 2 * cos(fov / 2.)


def calcLaserPt(d, theta, phi):
    """

    :param d:
    :param theta:
    :param phi:
    :return:
    """
    return np.array([-d * cos(phi) * sin(theta),
                     -d * sin(phi),
                     d * cos(phi) * cos(theta)])


def ptInImPlane(pt):
    """
        Given 3d point in camera frame, calculate its 2d position on image plane,
        i.e. 3d point with z = image plane depth.
        :param pt: coordinate in 3d camera frame
        """
    return -im_sensor_size[0] / 2 < pt[0] < im_sensor_size[0] / 2 \
           and -im_sensor_size[1] / 2 < pt[1] < im_sensor_size[1] / 2


def calcRes(l, theta, phi, fov, d):
    """
    Function to calculate resolution, defined by the projective length of a
    unit-length line segment on the laser ray, given the camera's field of view
    and configuration of laser.
    We make the following assumptions:
    1. Camera is perfect pin-hole.
    2. laser origin has zero depth in camera frame.
    :param l: baseline, distance between camera origin and laser origin
    :param theta: angle between laser plane and line connecting laser and
    camera origin
    :param phi: elevation angle of laser ray
    :param fov: field of view of camera in degrees.
    :param d: distance
    :return: resolution
    """
    laser_origin = np.array([l, 0., 0.])
    ray1 = calcLaserPt(d, theta, phi)
    ray2 = calcLaserPt(d + 0.001, theta, phi)
    pt1_cam = laser_origin + ray1
    pt1_im_plane = pt1_cam[0:2] / pt1_cam[2] * calcImPlaneZ(fov)
    pt2_cam = laser_origin + ray2
    pt2_im_plane = pt2_cam[0:2] / pt2_cam[2] * calcImPlaneZ(fov)

    if not (ptInImPlane(pt1_im_plane) and ptInImPlane(pt2_im_plane)):
        return -1
    else:
        return np.linalg.norm(pt2_im_plane - pt1_im_plane)


def calcResLine(l, theta, phi, fov):
    """
    Calculate average resolution along a line, from 3 to 25 centimeters
    :param l:
    :param theta:
    :param phi:
    :param fov:
    :return:
    """
    gaussian_weights = signal.gaussian(29, std=10)[7:]
    res_arr = np.zeros(22)
    for i in range(22):
        res = calcRes(l, theta, phi, fov, i * 0.01 + 0.03)
        if res < 0:
            return -1
        else:
            res_arr[i] = res
    avr_res = res_arr.dot(gaussian_weights) / np.sum(gaussian_weights)
    return avr_res


def calcShpericalSectorArea(fov):
    return 1 - cos(fov/2)


if __name__ == '__main__':
    l = 0.02
    d = 0.1
    # analyze theta and fov at d = 0.1

    thetas = np.linspace(0, pi * 4 / 9, 100) # 0 ~ 80 degrees
    fovs = np.linspace(pi / 3, pi / 9 * 8, 100) # 60 ~ 160 degrees
    X, Y = np.meshgrid(thetas, fovs)
    Z = np.zeros((100, 100))
    for i in range(100):
        for j in range(100):
            res = calcResLine(l, thetas[i], 0, fovs[j])
            if res > 0:
                Z[j,i] = res
            else:
                Z[j,i] = np.nan

    fig = plt.figure()
    ax = fig.gca(projection="3d")
    surf = ax.plot_surface(X / pi * 180, Y / pi * 180, Z, cmap=cm.coolwarm,
                           vmin=np.nanmin(Z), vmax=np.nanmax(Z))
    ax.set_xlabel("Angle")
    ax.set_ylabel("FoV")
    ax.set_zlabel("Resolution")
    ax.set_zlim(0, 1e-5)

    # at 160 degrees, plot resolution to distance at given theta
    fig2 = plt.figure()
    res_30 = np.zeros(100)
    res_60 = np.zeros(100)
    res_80 = np.zeros(100)
    d = np.linspace(0.03, 0.3, 100)
    for i in range(100):
        res_30[i] = calcRes(l, pi / 6, 0, 160 / 180.0 * pi, d[i])
        res_60[i] = calcRes(l, pi / 3, 0, 160 / 180.0 * pi, d[i])
        res_80[i] = calcRes(l, 80.0 / 180 * pi, 0, 160 / 180.0 * pi, d[i])
    plt.plot(d, res_30, label="30 Degrees")
    plt.plot(d, res_60, label="60 Degrees")
    plt.plot(d, res_80, label="80 Degrees")
    gaussian_weights = signal.gaussian(29, std=10)[7:] * 2e-5
    plt.plot(np.linspace(0.03, 0.25, 22), gaussian_weights, label="Weights")
    plt.ylim(0, 5e-5)
    plt.xlabel("distance (m)")
    plt.ylabel("resolution")
    plt.legend()

    # resolutin to information
    theta = pi/4
    res_45 = np.zeros(100)
    info_45 = np.zeros(100)
    for i in range(100):
        res_45[i] = calcResLine(l, theta, 0, fovs[i])
        info_45[i] = calcShpericalSectorArea(fovs[i])
    fig3 = plt.figure()
    plt.plot(fovs / pi * 180, res_45, label="Resolution")
    plt.plot(fovs / pi * 180, info_45 * 1e-5, label="Visible Area")
    plt.legend()
    plt.ylim(0, 1e-5)
    plt.xlabel("FoV (degrees)")
    plt.ylabel("Scaled Value")
    plt.show()
