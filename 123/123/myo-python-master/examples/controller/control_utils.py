import numpy as np
import numba


def jit_decorator(func):
    if True:
        return numba.jit(nopython=True, cache=True)(func)
    else:
        return func


@jit_decorator
def quat_mul(a, b):
    x1, y1, z1, w1 = a[0], a[1], a[2], a[3]
    x2, y2, z2, w2 = b[0], b[1], b[2], b[3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)
    return np.array([x, y, z, w])


@jit_decorator
def quat2mat(quaternion):
    """
    Converts given quaternion to matrix.
    Args:
        quaternion (np.ndarray): (..., 4) tensor where the final dim is (x,y,z,w) quaternion
    Returns:
        np.ndarray: (..., 3, 3) tensor whose final two dimensions are 3x3 rotation matrices
    """
    # convert quat convention
    inds = np.array([3, 0, 1, 2])
    input_shape = quaternion.shape[:-1]
    q = quaternion.reshape((-1, 4))[:, inds]  # [bs, (w, x, y, z)]

    # Conduct dot product
    n = np.matmul(q[:, None], q[:, :, None]).squeeze(-1).squeeze(-1)  # shape (-1)
    idx = np.nonzero(n)[0]
    q_ = q.copy()  # Copy so we don't have inplace operations that fail to backprop
    q_[idx, :] = q[idx, :] * np.sqrt(2.0 / n[idx][:, None])  # [bs, 4]

    # Conduct outer product
    q2 = np.matmul(q_[:, :, None], q_[:, None])  # shape (-1, 4 ,4)
    # Create return array
    ret = np.eye(3).reshape(1, 3, 3).repeat(int(np.prod(input_shape)), axis=0)
    ret[idx, :, :] = np.stack([
        np.stack([1.0 - q2[idx, 2, 2] - q2[idx, 3, 3], q2[idx, 1, 2] - q2[idx, 3, 0], q2[idx, 1, 3] + q2[idx, 2, 0]], axis=-1),
        np.stack([q2[idx, 1, 2] + q2[idx, 3, 0], 1.0 - q2[idx, 1, 1] - q2[idx, 3, 3], q2[idx, 2, 3] - q2[idx, 1, 0]], axis=-1),
        np.stack([q2[idx, 1, 3] - q2[idx, 2, 0], q2[idx, 2, 3] + q2[idx, 1, 0], 1.0 - q2[idx, 1, 1] - q2[idx, 2, 2]], axis=-1),
    ], axis=1)

    # Reshape and return output
    ret = ret.reshape(list(input_shape) + [3, 3])
    return ret


@jit_decorator
def mat2quat(rmat):
    """
    Converts given rotation matrix to quaternion.

    Args:
        rmat (np.ndarray): 3x3 rotation matrix

    Returns:
        np.array: (x,y,z,w) float quaternion angles
    """
    M = np.asarray(rmat).astype(float)[:3, :3]

    m00 = M[0, 0]
    m01 = M[0, 1]
    m02 = M[0, 2]
    m10 = M[1, 0]
    m11 = M[1, 1]
    m12 = M[1, 2]
    m20 = M[2, 0]
    m21 = M[2, 1]
    m22 = M[2, 2]
    # symmetric matrix K
    K = np.array(
        [
            [m00 - m11 - m22, float(0.0), float(0.0), float(0.0)],
            [m01 + m10, m11 - m00 - m22, float(0.0), float(0.0)],
            [m02 + m20, m12 + m21, m22 - m00 - m11, float(0.0)],
            [m21 - m12, m02 - m20, m10 - m01, m00 + m11 + m22],
        ]
    )
    K /= 3.0
    # quaternion is Eigen vector of K that corresponds to largest eigenvalue
    w, V = np.linalg.eigh(K)
    inds = np.array([3, 0, 1, 2])
    q1 = V[inds, np.argmax(w)]
    if q1[0] < 0.0:
        np.negative(q1, q1)
    inds = np.array([1, 2, 3, 0])
    return q1[inds]


@jit_decorator
def quat_conjugate(q):
    conjugate = np.zeros_like(q)
    conjugate[:3] = -q[:3]
    conjugate[3] = q[3]
    return conjugate


@jit_decorator
def quat_inv(q):
    conjugate = quat_conjugate(q)
    norm_q = np.linalg.norm(q)
    inv_q = conjugate / (norm_q + 1e-6)
    return inv_q


@jit_decorator
def orientation_error(desired, current):
    """
    This function calculates a 3-dimensional orientation error vector for use in the
    impedance controller. It does this by computing the delta rotation between the
    inputs and converting that rotation to exponential coordinates (axis-angle
    representation, where the 3d vector is axis * angle).
    See https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation for more information.
    Optimized function to determine orientation error from matrices

    Args:
        desired (np.ndarray): 2d array representing target orientation matrix
        current (np.ndarray): 2d array representing current orientation matrix

    Returns:
        np.array: 2d array representing orientation error as a matrix
    """
    rc1 = current[0:3, 0]
    rc2 = current[0:3, 1]
    rc3 = current[0:3, 2]
    rd1 = desired[0:3, 0]
    rd2 = desired[0:3, 1]
    rd3 = desired[0:3, 2]
    error = 0.5 * (np.cross(rc1, rd1) + np.cross(rc2, rd2) + np.cross(rc3, rd3))
    return error


@jit_decorator
def axisangle2quat(vec, eps=1e-6):
    """
    Converts scaled axis-angle to quat.
    Args:
        vec (np.ndarray): (..., 3) tensor where final dim is (ax,ay,az) axis-angle exponential coordinates
        eps (float): Stability value below which small values will be mapped to 0

    Returns:
        tensor: (..., 4) tensor where final dim is (x,y,z,w) vec4 float quaternion
    """
    # store input shape and reshape
    input_shape = vec.shape[:-1]
    vec = vec.reshape((-1, 3))

    # Grab angle
    angle = np.linalg.norm(vec, axis=-1, keepdims=True)

    # Create return array
    quat = np.zeros((int(np.prod(input_shape)), 4))
    quat[:, 3] = 1.0

    # Grab indexes where angle is not zero an convert the input to its quaternion form
    idx = angle.reshape(-1) > eps  # torch.nonzero(angle).reshape(-1)
    quat[idx, :] = np.concatenate([vec[idx, :] * np.sin(angle[idx, :] / 2.0) / angle[idx, :],
                                   np.cos(angle[idx, :] / 2.0)], axis=-1)

    # Reshape and return output
    quat = quat.reshape(list(input_shape) + [4, ])
    return quat

