import numpy as np
import quaternion

def multiply_quaternions(q1, q2):
    """
    Multiplies two quaternions together and returns the result as a quaternion array.

    Args:
        q1: A NumPy array representing the first quaternion in the format (w, x, y, z).
        q2: A NumPy array representing the second quaternion in the format (w, x, y, z).

    Returns:
        A NumPy array representing the result of the quaternion multiplication in the format (w, x, y, z).
    """

    q1_quat = np.quaternion(q1[0], q1[1], q1[2], q1[3])
    q2_quat = np.quaternion(q2[0], q2[1], q2[2], q2[3])
    result_quat = q1_quat * q2_quat
    result_array = np.array([result_quat.w, result_quat.x, result_quat.y, result_quat.z])
    return result_array

def quaternion_conjugate(q):
    """
    Computes the conjugate of a quaternion and returns the result as a NumPy array.

    Args:
        q: A NumPy array representing the quaternion in the format (w, x, y, z).

    Returns:
        A NumPy array representing the conjugate of the quaternion in the format (w, x, y, z).
    """
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    return q_conj

def quaternion_from_matrix(matrix):
    """
    Converts a NumPy array matrix to a quaternion and returns the result as a NumPy array.

    Args:
        matrix: A NumPy array representing the matrix.

    Returns:
        A NumPy array representing the quaternion in the format (w, x, y, z).
    """
    q = np.empty((4, ))
    M = matrix[:3, :3]
    t = np.trace(M)

    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        q[0] = 0.25 * s
        q[1] = (M[2, 1] - M[1, 2]) / s
        q[2] = (M[0, 2] - M[2, 0]) / s
        q[3] = (M[1, 0] - M[0, 1]) / s
    elif (M[0, 0] > M[1, 1]) and (M[0, 0] > M[2, 2]):
        s = np.sqrt(1.0 + M[0, 0] - M[1, 1] - M[2, 2]) * 2.0
        q[0] = (M[2, 1] - M[1, 2]) / s
        q[1] = 0.25 * s
        q[2] = (M[0, 1] + M[1, 0]) / s
        q[3] = (M[0, 2] + M[2, 0]) / s
    elif M[1, 1] > M[2, 2]:
        s = np.sqrt(1.0 + M[1, 1] - M[0, 0] - M[2, 2]) * 2.0
        q[0] = (M[0, 2] - M[2, 0]) / s
        q[1] = (M[0, 1] + M[1, 0]) / s
        q[2] = 0.25 * s
        q[3] = (M[1, 2] + M[2, 1]) / s
    else:
        s = np.sqrt(1.0 + M[2, 2] - M[0, 0] - M[1, 1]) * 2.0
        q[0] = (M[1, 0] - M[0, 1]) / s
        q[1] = (M[0, 2] + M[2, 0]) / s
        q[2] = (M[1, 2] + M[2, 1]) / s
        q[3] = 0.25 * s

    return q