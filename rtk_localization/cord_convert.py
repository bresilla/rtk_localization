import numpy as np
import rclpy
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
from geometry_msgs.msg import Pose
from handy_msgs.srv import Gps2Ecef, Ecef2Gps, Ecef2Enu, Enu2Ecef, Gps2Enu, Enu2Gps


wgs84_a = 6378137.0 # Semi-major Axis [m]
wgs84_f = 1./298.257223563 # Flattening
wgs84_omega_E = 7292115.0e-11 # Angular velocity of the Earth [rad/s]
wgs84_omega_E_GPS = 7292115.1467e-11 # Angular velocity of the Earth [rad/s]
                              # According to ICD-GPS-200

wgs84_GM = 3986004.418e8 # Earth's Gravitational Constant [m^3/s^2]
                   # (mass of earth's atmosphere included)

wgs84_GM_GPS = 3986005.0e8 # The WGS 84 GM value recommended for GPS receiver usage 
                     # by the GPS interface control document (ICD-GPS-200) 
                     # differs from the current refined WGS 84 GM value.
                     #
                     # Details for this difference can be read in the WGS84 
                     # reference: 3.2.3.2 "Special Considerations for GPS"

# Table 3.3: WGS 84 Ellipsoid Derived Geometric Constants
wgs84_b = 6356752.3142 # Semi-minor axis [m]
wgs84_ecc = 8.1819190842622e-2 # First eccentricity
wgs84_ecc_sqrd = 6.69437999014e-3 # First eccentricity squared



def _input_check_Nx1(x):
    """
    Check x to be of dimension Nx1 and reshape it as a 1-D array

    Adhika Lie
    """
    x = np.atleast_1d(x)
    theSize = np.shape(x)

    if(len(theSize) > 1):
        # 1. Input must be of size N x 1
        if ((theSize[0] != 1) & (theSize[1] != 1)):
            raise ValueError('Not an N x 1 array')
        # 2. Make it into a 1-D array
        x = x.reshape(np.size(x))
    elif (theSize[0] == 1):
        x = x[0]

    return x, np.size(x)


def _input_check_Nx3(x):
    """
    Check x to be of dimension Nx3

    Adhika Lie
    """
    x = np.atleast_2d(x)
    theSize = np.shape(x)

    if(len(theSize) > 1):
        # 1. Input must be of size N x 3
        if ((theSize[0] != 3) & (theSize[1] != 3)):
            raise ValueError('Not a N x 3 array')
        # 2. Make it into a Nx3 array
        if (theSize[1] != 3):
            x = x.T
        N = x.shape[0]
        # 3. If N == 1, make it into a 1-D array
        if (x.shape[0] == 1):
            x = x.reshape(x.shape[1])

    return x, N


def _input_check_Nx3x3(x):
    """
    Check x to be of dimension Nx3x3

    Jacob Niehus
    """
    theSize = np.shape(x)
    N = 1

    if(len(theSize) > 2):
        # 1. Input must be of size N x 3
        if (3, 3) not in (theSize[:2], theSize[-2:]):
            raise ValueError('Not a N x 3 x 3 array')
        # 2. Make it into a Nx3x3 array
        if (theSize[1:] != (3, 3)):
            x = np.rollaxis(x, -1)
        N = x.shape[0]
        # 3. If N == 2, make it into a 2-D array
        if (x.shape[0] == 1):
            x = x[0]
    elif(theSize != (3, 3)):
        raise ValueError('Not a 3 x 3 array')

    return x, N


def loadtxt2dic(filename):
    """
    Loads text file of key:value pairs into a dictionary.
    Usage notes:
    -Lines begining with '#' are treated as comments and skipped.
    -Blank lines are also skipped
    -Keys and values should be separated by '=' or ':', extra spaces are fine.
    -A matrix/scalar are stored floats ONLY if the text has a decimal

    Hamid M. (original)
    Adhika Lie
    """
    fid = open(filename, 'r')
    param = {}
    prev_line = ''

    for line in fid:
        # Remove Extra Spaces
        line = prev_line + line.strip()
        print(line)
        # Skip lines beginning with # or blank
        # Note: Python treats '' as False
        if(line.startswith('#') or line.startswith('\n') or (not line)):
            continue

        # If line ends with a comma, it continues to the next line.
        if(line.endswith(',')):
            prev_line = line.strip()
            continue
        else:
            prev_line = ''

        # Split item
        item = line.split('#', 1)[0].strip()  # May have comment after the line
        item = item.replace(':', ' ').replace('=', ' ').split(None, 1)

        if(len(item) == 0):
            continue

        try:
            param[item[0]] = eval(item[1])

            if(type(eval(item[1])) == list):
                param[item[0]] = np.array(eval(item[1]))

        except NameError:
            param[item[0]] = item[1]

    fid.close()
    return param



def angle2dcm(rotAngle1, rotAngle2, rotAngle3, input_unit='rad',
              rotation_sequence='ZYX', output_type='ndarray'):
    """
    This function converts Euler Angle into Direction Cosine Matrix (DCM).
    The DCM is described by three sucessive rotation rotAngle1, rotAngle2, and
    rotAngle3 about the axes described by the rotation_sequence.

    The default rotation_sequence='ZYX' is the aerospace sequence and rotAngle1
    is the yaw angle, rotAngle2 is the pitch angle, and rotAngle3 is the roll
    angle. In this case DCM transforms a vector from the locally level
    coordinate frame (i.e. the NED frame) to the body frame.

    This function can batch process a series of rotations (e.g., time series
    of Euler angles).

    Parameters
    ----------
    rotAngle1, rotAngle2, rotAngle3 : angles {(N,), (N,1), or (1,N)}
            They are a sequence of angles about successive axes described by
            rotation_sequence.
    input_unit : {'rad', 'deg'}, optional
            Rotation angles. Default is 'rad'.
    rotation_sequence : {'ZYX'}, optional
            Rotation sequences. Default is 'ZYX'.
    output_type : {'ndarray','matrix'}, optional
            Output type. Default is 'ndarray'.

    Returns
    --------
    C : {3x3} Direction Cosine Matrix

    Notes
    -----
    Programmer:    Adhika Lie
    Created:    	 May 03, 2011
    Last Modified: January 12, 2016
    """
    rotAngle1, N1 = _input_check_Nx1(rotAngle1)
    rotAngle2, N2 = _input_check_Nx1(rotAngle2)
    rotAngle3, N3 = _input_check_Nx1(rotAngle3)

    if(N1 != N2 or N1 != N3):
        raise ValueError('Inputs are not of same dimensions')
    if(N1 > 1 and output_type != 'ndarray'):
        raise ValueError('Matrix output requires scalar inputs')

    R3 = np.zeros((N1, 3, 3))
    R2 = np.zeros((N1, 3, 3))
    R1 = np.zeros((N1, 3, 3))

    if(input_unit == 'deg'):
        rotAngle1 = np.deg2rad(rotAngle1)
        rotAngle2 = np.deg2rad(rotAngle2)
        rotAngle3 = np.deg2rad(rotAngle3)

    R3[:, 2, 2] = 1.0
    R3[:, 0, 0] = np.cos(rotAngle1)
    R3[:, 0, 1] = np.sin(rotAngle1)
    R3[:, 1, 0] = -np.sin(rotAngle1)
    R3[:, 1, 1] = np.cos(rotAngle1)

    R2[:, 1, 1] = 1.0
    R2[:, 0, 0] = np.cos(rotAngle2)
    R2[:, 0, 2] = -np.sin(rotAngle2)
    R2[:, 2, 0] = np.sin(rotAngle2)
    R2[:, 2, 2] = np.cos(rotAngle2)

    R1[:, 0, 0] = 1.0
    R1[:, 1, 1] = np.cos(rotAngle3)
    R1[:, 1, 2] = np.sin(rotAngle3)
    R1[:, 2, 1] = -np.sin(rotAngle3)
    R1[:, 2, 2] = np.cos(rotAngle3)

    if rotation_sequence == 'ZYX':
        try:
            # Equivalent to C = R1.dot(R2.dot(R3)) for each of N inputs but
            # implemented efficiently in C extension
            C = np.einsum('nij, njk, nkm -> nim', R1, R2, R3)
        except AttributeError:
            # Older NumPy without einsum
            C = np.zeros((N1, 3, 3))
            for i, (R1, R2, R3) in enumerate(zip(R1, R2, R3)):
                C[i] = R1.dot(R2.dot(R3))
    else:
        raise NotImplementedError('Rotation sequences other than ZYX are not currently implemented')

    if(N1 == 1):
        C = C[0]
    if(output_type == 'matrix'):
        C = np.matrix(C)

    return C


def dcm2angle(C, output_unit='rad', rotation_sequence='ZYX'):
    """
    This function converts a Direction Cosine Matrix (DCM) into the three
    rotation angles.
    The DCM is described by three sucessive rotation rotAngle1, rotAngle2, and
    rotAngle3 about the axes described by the rotation_sequence.

    The default rotation_sequence='ZYX' is the aerospace sequence and rotAngle1
    is the yaw angle, rotAngle2 is the pitch angle, and rotAngle3 is the roll
    angle. In this case DCM transforms a vector from the locally level
    coordinate frame (i.e. the NED frame) to the body frame.

    This function can batch process a series of rotations (e.g., time series
    of direction cosine matrices).

    Parameters
    ----------
    C : {(3,3), (N,3,3), or (3,3,N)}
        direction consine matrix that rotates the vector from the first frame
        to the second frame according to the specified rotation_sequence.
    output_unit : {'rad', 'deg'}, optional
            Rotation angles. Default is 'rad'.
    rotation_sequence : {'ZYX'}, optional
            Rotation sequences. Default is 'ZYX'.

    Returns
    -------
    rotAngle1, rotAngle2, rotAngle3 :  angles
            They are a sequence of angles about successive axes described by
            rotation_sequence.

    Notes
    -----
    The returned rotAngle1 and 3 will be between   +/- 180 deg (+/- pi rad).
    In contrast, rotAngle2 will be in the interval +/- 90 deg (+/- pi/2 rad).

    In the 'ZYX' or '321' aerospace sequence, that means the pitch angle
    returned will always be inside the closed interval +/- 90 deg (+/- pi/2 rad).
    Applications where pitch angles near or larger than 90 degrees in magnitude
    are expected should used alternate attitude parameterizations like
    quaternions.
    """
    C, N = _input_check_Nx3x3(C)

    if(rotation_sequence == 'ZYX'):
        rotAngle1 = np.arctan2(C[..., 0, 1], C[..., 0, 0])   # Yaw
        rotAngle2 = -np.arcsin(C[..., 0, 2])  # Pitch
        rotAngle3 = np.arctan2(C[..., 1, 2], C[..., 2, 2])  # Roll

    else:
        raise NotImplementedError('Rotation sequences other than ZYX are not currently implemented')

    if(output_unit == 'deg'):
        rotAngle1 = np.rad2deg(rotAngle1)
        rotAngle2 = np.rad2deg(rotAngle2)
        rotAngle3 = np.rad2deg(rotAngle3)

    return rotAngle1, rotAngle2, rotAngle3


def omega2rates(pitch, roll, input_unit='rad',
                euler_angles_order='roll_pitch_yaw', output_type='ndarray'):
    """
    This function is used to create the transformation matrix to go from:
	    [p, q, r] --> [roll_rate, pitch_rate, yaw_rate]

    where pqr are xyz body rotation-rate measurements expressed in body frame.
    Yaw, pitch, and roll are the Euler angles.  We assume the Euler angles are
    3-2-1 (i.e Yaw -> Pitch -> Roll) transformations that go from navigation-
    frame to body-frame.

    Parameters
    ----------
    pitch : pitch angle, units of input_unit.
    roll  : roll angle , units of input_unit.
    input_unit : units for input angles {'rad', 'deg'}, optional
    euler_angles_order : {'roll_pitch_yaw', 'yaw_pitch_roll'}, optional
        Assumed order of Euler Angles attitude state vector (see ``Notes``).
    output_type : {'ndarray' or 'matrix'}, optional
        Numpy array (default) or matrix

    Returns
    -------
    R : transformation matrix, from xyz body-rate to Euler angle-rates
        numpy 'output_type' 3x3 (Note: default return variable is an ARRAY,
        not a matrix)

    Notes
    -----
    Since the returned transformation matrix is used to transform one vector
    to another, the assumed attitude variables order matters.
    The ``euler_angles_order`` parameter can be used to specify the assumed
    order.

    The difference is demonstrated by example:

        By default euler_angles_order='roll_pitch_yaw'
        R = omega2rates(pitch, roll)
        [ roll_rate]         [omega_x]
        [pitch_rate] = dot(R,[omega_y])
        [  yaw_rate]         [omega_z]

        Now assume our attitude state is [yaw, pitch, roll].T
        R = omega2rates(pitch, roll, euler_angles_order='yaw_pitch_roll')
        [ yaw_rate]          [omega_x]
        [pitch_rate] = dot(R,[omega_y])
        [ roll_rate]         [omega_z]

    References
    ----------
    [1] Equation 2.74, Aided Navigation: GPS with High Rate Sensors,
        Jay A. Farrel 2008

    [2] omega2rates.m function at:
    http://www.gnssapplications.org/downloads/chapter7/Chapter7_GNSS_INS_Functions.tar.gz
    """
    # Apply necessary unit transformations.
    if input_unit == 'rad':
        pitch_rad, roll_rad = pitch, roll
    elif input_unit == 'deg':
        pitch_rad, roll_rad = np.radians([pitch, roll])

    # Build transformation matrix.
    s_r, c_r = np.sin( roll_rad), np.cos( roll_rad)
    s_p, c_p = np.sin(pitch_rad), np.cos(pitch_rad)
    
    # Check for singularities (i.e. pitch near 90 degrees)
    singular_tol = 1e-2; # flags anything between [90 +/- .5 deg]
    if abs(c_p) < singular_tol:
        print('WARNING (omega2rates): Operating near pitch = 90 deg singularity.  NaN returned. ')
        return np.nan

    if euler_angles_order == 'roll_pitch_yaw':
        R = np.array(
           [[  1, s_r*s_p/c_p,  c_r*s_p/c_p],
            [  0, c_r        , -s_r        ],
            [  0, s_r/c_p    ,  c_r/c_p    ]], dtype=float)
    elif euler_angles_order == 'yaw_pitch_roll':
        R = np.array(
           [[  0, s_r/c_p    ,  c_r/c_p    ],
            [  0, c_r        , -s_r        ],
            [  1, s_r*s_p/c_p,  c_r*s_p/c_p]], dtype=float)

    if output_type == 'ndarray':
        pass
    elif output_type=='matrix':
        R = np.matrix(R)
    else:
        print("WARNING (omega2rates): Unrecognized 'output_type' requested.")
        print("NaN is returned.")
        return np.nan
        
    return R
    

def angle2quat(rotAngle1,rotAngle2,rotAngle3,
                input_unit='rad',rotation_sequence='ZYX'):
    """
    Convert a sequence of rotation angles to an equivalent unit quaternion
    
    This function can take inputs in either degree or radians, and can also 
    batch process a series of rotations (e.g., time series of Euler angles).
    By default this function assumes aerospace rotation sequence but can be 
    changed using the ``rotation_sequence`` keyword argument.
    
    Parameters
    ----------
    rotAngle1, rotAngle2, rotAngle3 : {(N,), (N,1), or (1,N)}
        They are a sequence of angles about successive axes described by rotation_sequence.
    input_unit : {'rad', 'deg'}, optional
        Rotation angles. Default is 'rad'.
    rotation_sequence : {'ZYX'}, optional 
        Rotation sequences. Default is 'ZYX'.
    
    Returns
    -------
    q0 : {(N,)} array like scalar componenet of the quaternion
    qvec : {(N,3)} array like vector component of the quaternion
    
    Notes
    -----
    Convert rotation angles to unit quaternion that transforms a vector in F1 to
    F2 according to
    
    :math:`v_q^{F2} = q^{-1} \otimes v_q^{F1} \otimes q`
    
    where :math:`\otimes` indicates the quaternion multiplcation and :math:`v_q^F`
    is a pure quaternion representation of the vector :math:`v_q^F`. The scalar
    componenet of :math:`v_q^F` is zero.
    For aerospace sequence ('ZYX'): rotAngle1 = psi, rotAngle2 = the,
    and rotAngle3 = phi
    
    Examples
    --------
    >>> import numpy as np
    >>> from navpy import angle2quat
    >>> psi = 0
    >>> theta = np.pi/4.0
    >>> phi = np.pi/3.0
    >>> q0, qvec = angle2quat(psi,theta,phi)
    >>> q0
    0.80010314519126557
    >>> qvec
    array([ 0.46193977,  0.33141357, -0.19134172])
    
    >>> psi = [10, 20, 30]
    >>> theta = [30, 40, 50]
    >>> phi = [0, 5, 10]
    >>> q0, qvec = angle2quat(psi,theta,phi,input_unit = 'deg')
    >>> q0
    array([ 0.96225019,  0.92712639,  0.88162808])
    >>> qvec
    array([[-0.02255757,  0.25783416,  0.08418598],
           [-0.01896854,  0.34362114,  0.14832854],
           [-0.03266701,  0.4271086 ,  0.19809857]])
    """
    
    # INPUT CHECK
    rotAngle1,N1 = _input_check_Nx1(rotAngle1)
    rotAngle2,N2 = _input_check_Nx1(rotAngle2)
    rotAngle3,N3 = _input_check_Nx1(rotAngle3)
    
    if( (N1!=N2) | (N1!=N3) | (N2!=N3) ):
        raise ValueError('Inputs are not of same dimensions')
    
    q0 = np.zeros(N1)
    qvec = np.zeros((N1,3))

    if(input_unit=='deg'):
        rotAngle1 = np.deg2rad(rotAngle1)
        rotAngle2 = np.deg2rad(rotAngle2)
        rotAngle3 = np.deg2rad(rotAngle3)
    
    rotAngle1 /= 2.0
    rotAngle2 /= 2.0
    rotAngle3 /= 2.0
    
    if(rotation_sequence=='ZYX'):
        q0[:] = np.cos(rotAngle1)*np.cos(rotAngle2)*np.cos(rotAngle3) + \
                np.sin(rotAngle1)*np.sin(rotAngle2)*np.sin(rotAngle3)

        qvec[:,0] = np.cos(rotAngle1)*np.cos(rotAngle2)*np.sin(rotAngle3) - \
            np.sin(rotAngle1)*np.sin(rotAngle2)*np.cos(rotAngle3)

        qvec[:,1] = np.cos(rotAngle1)*np.sin(rotAngle2)*np.cos(rotAngle3) + \
            np.sin(rotAngle1)*np.cos(rotAngle2)*np.sin(rotAngle3)

        qvec[:,2] = np.sin(rotAngle1)*np.cos(rotAngle2)*np.cos(rotAngle3) - \
            np.cos(rotAngle1)*np.sin(rotAngle2)*np.sin(rotAngle3)
    else:
        raise ValueError('rotation_sequence unknown')

    if(N1 == 1):
        q0 = q0[0]
        qvec = qvec.reshape(3,)
    return q0, qvec

def quat2angle(q0,qvec,output_unit='rad',rotation_sequence='ZYX'):
    """
    Convert a unit quaternion to the equivalent sequence of angles of rotation
    about the rotation_sequence axes.
    
    This function can take inputs in either degree or radians, and can also
    batch process a series of rotations (e.g., time series of quaternions).
    By default this function assumes aerospace rotation sequence but can be
    changed using the ``rotation_sequence`` keyword argument.
    
    Parameters
    ----------
    q0 : {(N,), (N,1), or (1,N)} array_like 
        Scalar componenet of the quaternion
    qvec : {(N,3),(3,N)} array_like 
        Vector component of the quaternion
    rotation_sequence : {'ZYX'}, optional
        Rotation sequences. Default is 'ZYX'.

    Returns
    -------
    rotAngle1, rotAngle2, rotAngle3 : {(N,), (N,1), or (1,N)} array_like
        They are a sequence of angles about successive axes described by
        rotation_sequence.
    output_unit : {'rad', 'deg'}, optional
        Rotation angles. Default is 'rad'.
    
    Notes
    -----
    Convert rotation angles to unit quaternion that transforms a vector in F1 to
    F2 according to
    
    :math:`v_q^{F2} = q^{-1} \otimes v_q^{F1} \otimes q`
    
    where :math:`\otimes` indicates the quaternion multiplcation and :math:`v_q^F`
    is a pure quaternion representation of the vector :math:`v_q^F`. The scalar
    componenet of :math:`v_q^F` is zero.
    For aerospace sequence ('ZYX'): rotAngle1 = psi, rotAngle2 = the, 
    and rotAngle3 = phi
    
    Examples
    --------
    >>> import numpy as np
    >>> from navpy import quat2angle
    >>> q0 = 0.800103145191266
    >>> qvec = np.array([0.4619398,0.3314136,-0.1913417])
    >>> psi, theta, phi = quat2angle(q0,qvec)
    >>> psi
    1.0217702360987295e-07
    >>> theta
    0.7853982192745731
    >>> phi
    1.0471976051067484
    
    >>> psi, theta, phi = quat2angle(q0,qvec,output_unit='deg')
    >>> psi
    5.8543122160542875e-06
    >>> theta
    45.00000320152342
    >>> phi
    60.000003088824108
    
    >>> q0 = [ 0.96225019,  0.92712639,  0.88162808]
    >>> qvec = np.array([[-0.02255757,  0.25783416,  0.08418598],\
                         [-0.01896854,  0.34362114,  0.14832854],\
                         [-0.03266701,  0.4271086 ,  0.19809857]])
    >>> psi, theta, phi = quat2angle(q0,qvec,output_unit='deg')
    >>> psi
    array([  9.99999941,  19.99999997,  29.9999993 ])
    >>> theta
    array([ 30.00000008,  39.99999971,  50.00000025])
    >>> phi
    array([ -6.06200867e-07,   5.00000036e+00,   1.00000001e+01])
    """
    q0, N0 = _input_check_Nx1(q0)
    qvec, Nvec = _input_check_Nx3(qvec)

    if(N0!=Nvec):
        raise ValueError('Inputs are not of same dimensions')
    if(N0 == 1):
        q1 = qvec[0]
        q2 = qvec[1]
        q3 = qvec[2]
    else:
        q1 = qvec[:,0]
        q2 = qvec[:,1]
        q3 = qvec[:,2]

    rotAngle1 = np.zeros(N0)
    rotAngle2 = np.zeros(N0)
    rotAngle3 = np.zeros(N0)

    if(rotation_sequence=='ZYX'):
        m11 = 2*q0**2 + 2*q1**2 - 1
        m12 = 2*q1*q2 + 2*q0*q3
        m13 = 2*q1*q3 - 2*q0*q2
        m23 = 2*q2*q3 + 2*q0*q1
        m33 = 2*q0**2 + 2*q3**2 - 1

        rotAngle1 = np.arctan2(m12,m11)
        rotAngle2 = np.arcsin(-m13)
        rotAngle3 = np.arctan2(m23,m33)
    else:
        raise ValueError('rotation_sequence unknown')

    if(output_unit=='deg'):
        rotAngle1 = np.rad2deg(rotAngle1)
        rotAngle2 = np.rad2deg(rotAngle2)
        rotAngle3 = np.rad2deg(rotAngle3)

    return rotAngle1, rotAngle2, rotAngle3

def quat2dcm(q0,qvec,rotation_sequence='ZYX',output_type='ndarray'):
    """
    Convert a single unit quaternion to one DCM
    
    Parameters
    ----------
    q0 : {(N,), (N,1), or (1,N)} array_like 
        Scalar componenet of the quaternion
    qvec : {(N,3),(3,N)} array_like 
        Vector component of the quaternion
    rotation_sequence : {'ZYX'}, optional
        Rotation sequences. Default is 'ZYX'.
    output_type : {'ndarray','matrix'}, optional
        Output is either numpy array (default) or numpy matrix.
            
    Returns
    -------
    C_N2B : direction consine matrix that rotates the vector from the first frame
            to the second frame according to the specified rotation_sequence.
        
    Examples
    --------
    >>> import numpy as np
    >>> from navpy import quat2dcm
    >>> q0 = 1
    >>> qvec = [0, 0, 0]
    >>> C = quat2dcm(q0,qvec)
    >>> C
    array([[ 1.,  0.,  0.],
           [ 0.,  1.,  0.],
           [ 0.,  0.,  1.]])
    
    >>> q0 = 0.9811
    >>> qvec = np.array([-0.0151, 0.0858, 0.1730])
    >>> C = quat2dcm(q0,qvec,output_type='matrix')
    >>> C
    matrix([[  9.25570440e-01,   3.36869440e-01,  -1.73581360e-01],
            [ -3.42051760e-01,   9.39837700e-01,   5.75800000e-05],
            [  1.63132160e-01,   5.93160200e-02,   9.84972420e-01]])
    """
    # Input check
    q0,N0 = _input_check_Nx1(q0)
    qvec,Nvec = _input_check_Nx3(qvec)

    if((N0!=1) | (Nvec!=1)):
        raise ValueError('Can only process 1 quaternion')
    
    q1 = qvec[0]
    q2 = qvec[1]
    q3 = qvec[2]
    if(rotation_sequence=='ZYX'):
        C_N2B = np.zeros((3,3))

        C_N2B[0,0] =  2*q0**2 - 1 + 2*q1**2
        C_N2B[1,1] =  2*q0**2 - 1 + 2*q2**2
        C_N2B[2,2] =  2*q0**2 - 1 + 2*q3**2

        C_N2B[0,1] = 2*q1*q2 + 2*q0*q3
        C_N2B[0,2] = 2*q1*q3 - 2*q0*q2

        C_N2B[1,0] = 2*q1*q2 - 2*q0*q3
        C_N2B[1,2] = 2*q2*q3 + 2*q0*q1

        C_N2B[2,0] = 2*q1*q3 + 2*q0*q2
        C_N2B[2,1] = 2*q2*q3 - 2*q0*q1
    else:
        raise ValueError('rotation_sequence unknown')

    if(output_type=='matrix'):
        C_N2B = np.asmatrix(C_N2B)

    return C_N2B

def dcm2quat(C,rotation_sequence='ZYX'):
    """
    Convert a DCM to a unit quaternion
    
    Parameters
    ----------
    C : direction consine matrix that rotates the vector from the first frame
        to the second frame according to the specified rotation_sequence.
        rotation_sequence: {'ZYX'}, optional. Rotation sequences. Default is 'ZYX'.
    
    Returns
    -------
    q0 : {(N,)} array_like 
        Scalar componenet of the quaternion
    qvec : {(N,3)} array_like 
        Vector component of the quaternion

    Examples
    --------
    >>> import numpy as np
    >>> from navpy import dcm2quat
    >>> C = np.array([[  9.25570440e-01,   3.36869440e-01,  -1.73581360e-01],
                      [ -3.42051760e-01,   9.39837700e-01,   5.75800000e-05],
                      [  1.63132160e-01,   5.93160200e-02,   9.84972420e-01]])
    >>> q0,qvec = dcm2quat(C)
    >>> q0
    0.98111933015306552
    >>> qvec
    array([-0.0150997 ,  0.08579831,  0.17299659])
    """
    
    if(C.shape[0]!=C.shape[1]):
        raise ValueError('Input is not a square matrix')
    if(C.shape[0]!=3):
        raise ValueError('Input needs to be a 3x3 array or matrix')

    qvec = np.zeros(3)
    q0 = 0.5*np.sqrt(C[0,0]+C[1,1]+C[2,2]+1)
    qvec[0] = (C[1,2]-C[2,1])/(4*q0)
    qvec[1] = (C[2,0]-C[0,2])/(4*q0)
    qvec[2] = (C[0,1]-C[1,0])/(4*q0)

    return q0,qvec

def qmult(p0,pvec,q0,qvec):
    """
    Quaternion Multiplications r = p x q
    
    Parameters
    ----------
    p0, q0 : {(N,)} array_like 
        Scalar componenet of the quaternion
    pvec, qvec : {(N,3)} array_like
        Vector component of the quaternion
    
    Returns
    -------
    r0 : {(N,)} array like scalar componenet of the quaternion
    rvec : {(N,3)} array like vector component of the quaternion
    
    Examples
    --------
    >>> import numpy as np
    >>> from navpy import qmult
    >>> p0, pvec = 0.701057, np.array([-0.69034553,  0.15304592,  0.09229596])
    >>> q0, qvec = 0.987228, np.array([ 0.12613659,  0.09199968,  0.03171637])
    >>> qmult(q0,qvec,p0,pvec)
    (0.76217346258977192, array([-0.58946236,  0.18205109,  0.1961684 ]))
    >>> s0, svec = 0.99879, np.array([ 0.02270747,  0.03430854, -0.02691584])
    >>> t0, tvec = 0.84285, np.array([ 0.19424161, -0.18023625, -0.46837843])
    >>> qmult(s0,svec,t0,tvec)
    (0.83099625967941704, array([ 0.19222498, -0.1456937 , -0.50125456]))
    >>> qmult([p0, s0],[pvec, svec],[q0, t0], [qvec, tvec])
    (array([ 0.76217346,  0.83099626]), array([[-0.59673664,  0.24912539,  0.03053588], [ 0.19222498, -0.1456937 , -0.50125456]]))
    """
    
    p0,Np = _input_check_Nx1(p0)
    q0,Nq = _input_check_Nx1(q0)
    if(Np!=Nq):
        raise ValueError('Inputs are not of the same dimension')
    
    pvec,Np = _input_check_Nx3(pvec)
    if(Np!=Nq):
        raise ValueError('Inputs are not of the same dimension')

    qvec,Nq = _input_check_Nx3(qvec)
    if(Np!=Nq):
        raise ValueError('Inputs are not of the same dimension')
    
    if(Np > 1):
        r0 = p0*q0 - np.sum(pvec*qvec,axis=1)
    else:
        r0 = p0*q0 - np.dot(pvec,qvec)

    rvec = p0.reshape(Np,1)*qvec + q0.reshape(Np,1)*pvec + np.cross(pvec,qvec)

    # For only 1-D input, make it into a flat 1-D array
    if(Np == 1):
        rvec = rvec.reshape(3)

    return r0,rvec

def llarate(VN,VE,VD,lat,alt,lat_unit='deg',alt_unit='m'):
    """
    Calculate Latitude, Longitude, Altitude Rate given locally tangent velocity
    
    Parameters
    ----------
    VN : {(N,)} array like earth relative velocity in the North direction, m/s
    VE : {(N,)} array like earth relative velocity in the East direction, m/s
    VD : {(N,)} array like earth relative velocity in the Down direction, m/s
    lat : {(N,)} array like latitudes, unit specified in lat_unit, default deg
    alt : {(N,)} array like altitudes, unit specified in alt_unit, default m
    
    Returns
    -------
    lla_dot : {(N,3)} np.array of latitude rate, longitude rate, altitude rate.
              The unit of latitude and longitude rate will be the same as the 
              unit specified by lat_unit and the unit of altitude rate will be 
              the same as alt_unit
    
    See Also
    --------
    earthrad : called by this method
    
    Examples
    --------
    >>> import numpy as np
    >>> from navpy import llarate
    >>> llarate(100,0,0,45.0,0) # Moving North at 100 m/s, location is at N45.0
    array([ 0.00089983,  0.        ,  0.        ])
    >>> # That output was in deg/sec
    >>> lat = [np.pi/4, -np.pi/6]
    >>> alt = [100.0, 50]
    >>> VN = [100, 0]
    >>> VE = [0, 100]
    >>> VD = [0, -5]
    >>> llarate(VN,VE,VD,lat,alt,lat_unit='rad')
    array([[  1.57047955e-05,   0.00000000e+00,   0.00000000e+00],\
           [  0.00000000e+00,   1.80887436e-05,   5.00000000e+00]])
    >>> # That output was in rad/sec
    """
    dim_check = 1
    VN, N1 = _input_check_Nx1(VN)
    VE, N2 = _input_check_Nx1(VE)
    if(N2!=N1):
        dim_check *= 0
    VD, N2 = _input_check_Nx1(VD)
    if(N2!=N1):
        dim_check *= 0
    lat,N2 = _input_check_Nx1(lat)
    if(N2!=N1):
        dim_check *= 0
    alt,N2 = _input_check_Nx1(alt)
    if(N2!=N1):
        dim_check *= 0
    if(dim_check==0):
        raise ValueError('Inputs are not of the same dimension')

    Rew, Rns = earthrad(lat,lat_unit=lat_unit)

    lla_dot = np.zeros((N1,3))
    if(lat_unit=='deg'):
        lla_dot[:,0] = np.rad2deg(VN/(Rns + alt))
        lla_dot[:,1] = np.rad2deg(VE/(Rew + alt)/np.cos(np.deg2rad(lat)))
        lla_dot[:,2] = -VD
    elif(lat_unit=='rad'):
        lla_dot[:,0] = VN/(Rns + alt)
        lla_dot[:,1] = VE/(Rew + alt)/np.cos(lat)
        lla_dot[:,2] = -VD

    if(N1==1):
        lla_dot = lla_dot.reshape(3)

    return lla_dot

def earthrate(lat, lat_unit = 'deg', model='wgs84'):
    """
    Calculate the earth rotation rate resolved on NED axes 
    given VN, VE, VD, lat, and alt.
    
    Paul Groves's Notation: :math:`\omega_{IE}^N`, Eq. (2.75), Ch. 2.3, pp. 44
    
    Parameters
    ----------
    lat : {(N,)} array like latitudes, unit specified in lat_unit, default deg
    
    Returns
    -------
    e : {(N,3)} np.array of the earth's rotation rate
        The unit is in rad/seconds.

    References
    ----------
    [1] P. Groves, GNSS, Inertial, and Integrated Navigation Systems, Artech House, 2008
    """
    if(lat_unit=='deg'):
        lat = np.deg2rad(lat)
    elif(lat_unit=='rad'):
        pass
    else:
        raise ValueError('Input unit unknown')

    lat,N = _input_check_Nx1(lat)

    e = np.zeros((N,3))
    if(model=='wgs84'):
        e[:,0] = wgs84_omega_E*np.cos(lat)
        e[:,1] = -wgs84_omega_E*np.sin(lat)
    else:
        raise ValueError('Model unknown')

    if(N==1):
        e = e.reshape(3)

    return e

def navrate(VN, VE, VD,lat, alt, lat_unit='deg', alt_unit='m', model='wgs84'):
    """
    Calculate navigation/transport rate given VN, VE, VD, lat, and alt.
    Navigation/transport rate is the angular velocity of the NED frame relative
    to the earth ECEF frame. 
    Paul Groves's Notation: :math:`\omega_{EN}^N`, Eq. (5.37), Ch. 5.3, pp. 131
    
    References
    ----------
    P. Groves, GNSS, Inertial, and Integrated Navigation Systems, Artech House, 2008
    
    Parameters
    ----------
    VN: {(N,)} array like earth relative velocity in the North direction, m/s
    VE: {(N,)} array like earth relative velocity in the East direction, m/s
    VD: {(N,)} array like earth relative velocity in the Down direction, m/s
    lat: {(N,)} array like latitudes, unit specified in lat_unit, default deg
    alt: {(N,)} array like altitudes, unit specified in alt_unit, default m
    
    Returns
    -------
    rho: {(N,3)} np.array of the transport rate.
    The unit is in rad/seconds.
    
    Calls
    -----
    earthrad
    """

    lat,N1 = _input_check_Nx1(lat)
    alt,N2 = _input_check_Nx1(alt)
    VN, N3 = _input_check_Nx1(VN)
    VE, N4 = _input_check_Nx1(VE)
    VD, N5 = _input_check_Nx1(VD)

    if((N1!=N2) or (N2!=N3) or (N1!=N3) or (N1!=N4) or (N4!=N5)):
        raise ValueError('Inputs are not of the same dimension')

    Rew, Rns = earthrad(lat,lat_unit=lat_unit)

    rho = np.zeros((N1,3))

    if(model=='wgs84'):
        rho[:,0] = VE/(Rew+alt)
        rho[:,1] = -VN/(Rns+alt)
        rho[:,2] = -VE*np.tan(np.deg2rad(lat))/(Rew+alt)
    else:
        raise ValueError('Model unknown')

    if(N1==1):
        rho = rho.reshape(3)

    return rho

def earthrad(lat, lat_unit='deg', model='wgs84'):
    """
    Calculate radius of curvature in the prime vertical (East-West) and 
    meridian (North-South) at a given latitude.

    Parameters
    ----------
    lat : {(N,)} array like latitude, unit specified by lat_unit, default in deg
    
    Returns
    -------
    R_N : {(N,)} array like, radius of curvature in the prime vertical (East-West)
    R_M : {(N,)} array like, radius of curvature in the meridian (North-South)
    
    Examples
    --------
    >>> import numpy as np
    >>> from navpy import earthrad
    >>> lat = 0
    >>> Rtransverse, Rmeridian = earthrad(lat)
    >>> Rtransverse
    6378137.0
    >>> Rmeridian
    6335439.3272928288
    >>> lat = [0, np.pi/2]
    >>> Rtransverse, Rmeridian = earthrad(lat,lat_unit='rad')
    >>> Rtransverse
    array([ 6378137.        ,  6399593.62575849])
    >>> Rmeridian
    array([ 6335439.32729283,  6399593.62575849])
    """
    if(lat_unit=='deg'):
        lat = np.deg2rad(lat)
    elif(lat_unit=='rad'):
        pass
    else:
        raise ValueError('Input unit unknown')

    if(model=='wgs84'):
        R_N = wgs84_a/(1-wgs84_ecc_sqrd*np.sin(lat)**2)**0.5
        R_M = wgs84_a*(1-wgs84_ecc_sqrd)/(1-wgs84_ecc_sqrd*np.sin(lat)**2)**1.5
    else:
        raise ValueError('Model unknown')
    
    return R_N, R_M

def lla2ecef(lat, lon, alt, latlon_unit='deg', alt_unit='m', model='wgs84'):
    """
    Convert Latitude, Longitude, Altitude, to ECEF position
    
    Parameters
    ----------
    lat : {(N,)} array like latitude, unit specified by latlon_unit, default in deg
    lon : {(N,)} array like longitude, unit specified by latlon_unit, default in deg
    alt : {(N,)} array like altitude, unit specified by alt_unit, default in m
    
    Returns
    -------
    ecef : {(N,3)} array like ecef position, unit is the same as alt_unit
    """
    lat,N1 = _input_check_Nx1(lat)
    lon,N2 = _input_check_Nx1(lon)
    alt,N3 = _input_check_Nx1(alt)
    
    if( (N1!=N2) or (N2!=N3) or (N1!=N3) ):
        raise ValueError('Inputs are not of the same dimension')
    
    if(model=='wgs84'):
        Rew,Rns = earthrad(lat,lat_unit=latlon_unit)
    else:
        Rew = wgs84_a 
    
    if(latlon_unit=='deg'):
        lat = np.deg2rad(lat)
        lon = np.deg2rad(lon)
    
    x = (Rew + alt)*np.cos(lat)*np.cos(lon)
    y = (Rew + alt)*np.cos(lat)*np.sin(lon)
    z = ( (1-wgs84_ecc_sqrd)*Rew + alt )*np.sin(lat)
    
    ecef = np.vstack((x,y,z)).T

    if(N1==1):
        ecef = ecef.reshape(3)

    return ecef

def ecef2lla(ecef, latlon_unit='deg'):
    """
    Calculate the Latitude, Longitude and Altitude of a point located on earth 
    given the ECEF Coordinates.
    
    References
    ----------
    .. [1] Jekeli, C.,"Inertial Navigation Systems With Geodetic
       Applications", Walter de Gruyter, New York, 2001, pp. 24
    
    Parameters
    ----------
    ecef : {(N,3)} array like input of ECEF coordinate in X, Y, and Z column, unit is meters
    latlon_unit : {('deg','rad')} specifies the output latitude and longitude unit
    
    Returns
    -------
    lat : {(N,)} array like latitude in unit specified by latlon_unit
    lon : {(N,)} array like longitude in unit specified by latlon_unit
    alt : {(N,)} array like altitude in meters
    """
    ecef,N = _input_check_Nx3(ecef)
    ecef = ecef.reshape(N,3)
    x = ecef[:,0]; y = ecef[:,1]; z = ecef[:,2]
    
    lon = np.arctan2(y,x)

    # Iteration to get Latitude and Altitude
    p = np.sqrt(x**2 + y**2)
    lat = np.arctan2(z,p*(1-wgs84_ecc_sqrd))

    err = np.ones(N)
    h = np.zeros(N)
    while(np.max(np.abs(err))>1e-10):
        Rew,Rns = earthrad(lat,lat_unit='rad')
        
        idx = (np.pi/2*np.ones(N) - np.abs(lat)) > 1e-3
        # For lat < 90 degrees
        h[idx] = np.divide(p[idx],np.cos(lat[idx])) - Rew[idx]
        # For lat == 90 degrees
        h[~idx] = np.divide(z[~idx],np.sin(lat[~idx])) - (1-wgs84_ecc_sqrd)*Rew[~idx]
        
        err = np.arctan2(z+wgs84_ecc_sqrd*Rew*np.sin(lat),p) - lat
        
        lat = lat + err
    
    if(latlon_unit=='deg'):
        lat = np.rad2deg(lat)
        lon = np.rad2deg(lon)
    if(N>1):
        return lat, lon, h
    else:
        return lat[0], lon[0], h[0]

def lla2ned(lat, lon, alt, lat_ref, lon_ref, alt_ref, latlon_unit='deg', alt_unit='m', model='wgs84'):
    """
    Convert Latitude, Longitude, Altitude to its resolution in the NED
    coordinate. The center of the NED coordiante is given by lat_ref, lon_ref,
    and alt_ref.
    
    For example, this can be used to convert GPS data to a local NED frame.
    
    Parameters
    ----------
    lat : {(N,)} array like latitude, unit specified by latlon_unit, default in deg
    lon : {(N,)} array like longitude, unit specified by latlon_unit, default in deg
    alt : {(N,)} array like altitude, unit specified by alt_unit, default in m
    
    lat_ref : Reference latitude, unit specified by latlon_unit, default in deg
    lon_ref : Reference longitude, unit specified by latlon_unit, default in deg
    alt : Reference altitude, unit specified by alt_unit, default in m
    
    Returns
    -------
    ned : {(N,3)} array like ecef position, unit is the same as alt_unit        
    """
    ecef  = lla2ecef(lat, lon, alt, latlon_unit=latlon_unit, 
                           alt_unit=alt_unit, model=model)
    ecef0 = lla2ecef(lat_ref, lon_ref, alt_ref,
                           latlon_unit=latlon_unit, 
                           alt_unit=alt_unit, model=model)
    ned  = ecef2ned(ecef-ecef0, lat_ref, lon_ref, alt_ref, 
                          latlon_unit=latlon_unit, alt_unit=alt_unit, model=model)
    return ned

def ned2lla(ned, lat_ref, lon_ref, alt_ref, latlon_unit='deg', alt_unit='m', model='wgs84'):
    """
    Calculate the Latitude, Longitude and Altitude of points given by NED coordinates
    where NED origin given by lat_ref, lon_ref, and alt_ref.

    Parameters
    ----------
    ned : {(N,3)} array like input of NED coordinate in N, E, and D column, unit is meters
    lat_ref : Reference latitude, unit specified by latlon_unit, default in deg
    lon_ref : Reference longitude, unit specified by latlon_unit, default in deg
    alt_ref : Reference altitude, unit specified by alt_unit, default in m
    latlon_unit : {('deg','rad')} specifies the output latitude and longitude unit
    
    Returns
    -------
    lat : {(N,)} array like latitude in unit specified by latlon_unit
    lon : {(N,)} array like longitude in unit specified by latlon_unit
    alt : {(N,)} array like altitude in meters

    Note
    ----
    This method is a wrapper on ned2ecef (add ecef of NED-origin) and ecef2lla.
    """

    ecef = ned2ecef(ned,lat_ref,lon_ref,alt_ref,latlon_unit=latlon_unit,
                                                alt_unit=alt_unit,
                                                model=model)
    # Add vector to ecef representation of NED-origin
    ecef_ref = lla2ecef(lat_ref, lon_ref, alt_ref, latlon_unit=latlon_unit,
                                                   alt_unit=alt_unit,
                                                   model=model)
    ecef += ecef_ref

    lla = ecef2lla(ecef, latlon_unit=latlon_unit)

    return lla


def ned2ecef(ned,lat_ref,lon_ref,alt_ref,latlon_unit='deg',alt_unit='m',model='wgs84'):
    """
    Transform a vector resolved in NED (origin given by lat_ref, lon_ref, and alt_ref)
    coordinates to its ECEF representation. 

    Parameters
    ----------
    ned : {(N,3)} input array, units of meters
    lat_ref : Reference latitude, unit specified by latlon_unit, default in deg
    lon_ref : Reference longitude, unit specified by latlon_unit, default in deg
    alt_ref : Reference altitude, unit specified by alt_unit, default in m
    
    Returns
    -------
    ecef : {(N,3)} array like ned vector, in the ECEF frame, units of meters

    Notes
    -----
    The NED vector is treated as a relative vector, and hence the ECEF representation
    returned is NOT converted into an absolute coordinate.  This means that the 
    magnitude of `ned` and `ecef` will be the same (bar numerical differences).
    
    Examples
    --------
    >>> import navpy
    >>> ned = [0, 0, 1]
    >>> lat_ref, lon_ref, alt_ref = 45.0, -93.0, 250.0 # deg, meters
    >>> ecef = navpy.ned2ecef(ned, lat_ref, lon_ref, alt_ref)
    >>> print("NED:", ned)
    >>> print("ECEF:", ecef)
    >>> print("Notice that 'down' is not same as 'ecef-z' coordinate.")
    """
    lat_ref,N1 = _input_check_Nx1(lat_ref)
    lon_ref,N2 = _input_check_Nx1(lon_ref)
    alt_ref,N3 = _input_check_Nx1(alt_ref)
    
    if( (N1!=1) or (N2!=1) or (N3!=1) ):
        raise ValueError('Reference Location can only be 1')
    
    ned,N = _input_check_Nx3(ned)

    ned = ned.T
    
    C = np.zeros((3,3))

    if(latlon_unit=='deg'):
        lat_ref = np.deg2rad(lat_ref)
        lon_ref = np.deg2rad(lon_ref)
    elif(latlon_unit=='rad'):
        pass
    else:
        raise ValueError('Input unit unknown')

    C[0,0]=-np.sin(lat_ref)*np.cos(lon_ref)
    C[0,1]=-np.sin(lat_ref)*np.sin(lon_ref)
    C[0,2]= np.cos(lat_ref)
    
    C[1,0]=-np.sin(lon_ref)
    C[1,1]= np.cos(lon_ref)
    C[1,2]= 0

    C[2,0]=-np.cos(lat_ref)*np.cos(lon_ref)
    C[2,1]=-np.cos(lat_ref)*np.sin(lon_ref)
    C[2,2]=-np.sin(lat_ref)

    # C defines transoformation: ned = C * ecef.  Hence used transpose.
    ecef = np.dot(C.T,ned)
    ecef = ecef.T

    if(N == 1):
        ecef = ecef.reshape(3)

    return ecef

def ecef2ned(ecef,lat_ref,lon_ref,alt_ref,latlon_unit='deg',alt_unit='m',model='wgs84'):
    """
    Transform a vector resolved in ECEF coordinate to its resolution in the NED
    coordinate. The center of the NED coordiante is given by lat_ref, lon_ref,
    and alt_ref.
    
    Parameters
    ----------
    ecef : {(N,3)} input vector expressed in the ECEF frame
    lat_ref : Reference latitude, unit specified by latlon_unit, default in deg
    lon_ref : Reference longitude, unit specified by latlon_unit, default in deg
    alt : Reference altitude, unit specified by alt_unit, default in m
    
    Returns
    -------
    ned : {(N,3)} array like ecef position, unit is the same as alt_unit
    
    Examples
    --------
    >>> import numpy as np
    >>> from navpy import ecef2ned
    >>> lat 
    """
    lat_ref,N1 = _input_check_Nx1(lat_ref)
    lon_ref,N2 = _input_check_Nx1(lon_ref)
    alt_ref,N3 = _input_check_Nx1(alt_ref)
    
    if( (N1!=1) or (N2!=1) or (N3!=1) ):
        raise ValueError('Reference Location can only be 1')
    
    ecef,N = _input_check_Nx3(ecef)

    ecef = ecef.T
    
    C = np.zeros((3,3))

    if(latlon_unit=='deg'):
        lat_ref = np.deg2rad(lat_ref)
        lon_ref = np.deg2rad(lon_ref)
    elif(latlon_unit=='rad'):
        pass
    else:
        raise ValueError('Input unit unknown')

    C[0,0]=-np.sin(lat_ref)*np.cos(lon_ref)
    C[0,1]=-np.sin(lat_ref)*np.sin(lon_ref)
    C[0,2]= np.cos(lat_ref)
	
    C[1,0]=-np.sin(lon_ref)
    C[1,1]= np.cos(lon_ref)
    C[1,2]= 0

    C[2,0]=-np.cos(lat_ref)*np.cos(lon_ref)
    C[2,1]=-np.cos(lat_ref)*np.sin(lon_ref)
    C[2,2]=-np.sin(lat_ref)

    ned = np.dot(C,ecef)
    ned = ned.T

    if(N == 1):
        ned = ned.reshape(3)

    return ned

def ned2enu(ned):
    """
    Transform a vector resolved in NED coordinate to its resolution in the ENU
    coordinate.
    
    Parameters
    ----------
    ned : {(N,3)} input vector expressed in the NED frame
    
    Returns
    -------
    enu : {(N,3)} array like ecef position, unit is the same as alt_unit
    
    Examples
    --------
    >>> import numpy as np
    >>> from navpy import ned2enu
    >>> ned = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    >>> enu = ned2enu(ned)
    >>> enu
    array([[ 0.,  1.,  0.],
           [ 1.,  0.,  0.],
           [ 0.,  0., -1.]])
    """
    ned,N = _input_check_Nx3(ned)

    ned = ned.T
    
    C = np.zeros((3,3))

    C[0,0]= 0
    C[0,1]= 1
    C[0,2]= 0
    
    C[1,0]= 1
    C[1,1]= 0
    C[1,2]= 0

    C[2,0]= 0
    C[2,1]= 0
    C[2,2]=-1

    enu = np.dot(C,ned)
    enu = enu.T

    if(N == 1):
        enu = enu.reshape(3)

    return enu

def gps_to_ecef(gps):
    lat, lon, alt = gps
    rad_lat = lat * (np.pi / 180.0)
    rad_lon = lon * (np.pi / 180.0)
    a = 6378137.0
    finv = 298.257223563
    f = 1 / finv
    e2 = 1 - (1 - f) * (1 - f)
    v = a / np.sqrt(1 - e2 * np.sin(rad_lat) * np.sin(rad_lat))
    x = (v + alt) * np.cos(rad_lat) * np.cos(rad_lon)
    y = (v + alt) * np.cos(rad_lat) * np.sin(rad_lon)
    z = (v * (1 - e2) + alt) * np.sin(rad_lat)
    return x, y, z

def gps_to_enu(gps, datum):
    fix_ecef = np.array(gps_to_ecef(gps))
    datum_ecef = np.array(gps_to_ecef(datum))
    # Calculate the differences in coordinates
    dx = fix_ecef[0] - datum_ecef[0]
    dy = fix_ecef[1] - datum_ecef[1]
    dz = fix_ecef[2] - datum_ecef[2]
    # Get observer's geodetic coordinates
    observer_lat = np.arctan2(datum_ecef[2], np.sqrt(datum_ecef[0]**2 + datum_ecef[1]**2))
    observer_lon = np.arctan2(datum_ecef[1], datum_ecef[0])
    # Calculate rotation matrix elements
    sin_lon = np.sin(observer_lon)
    cos_lon = np.cos(observer_lon)
    sin_lat = np.sin(observer_lat)
    cos_lat = np.cos(observer_lat)
    # Calculate rotation matrix
    rotation_matrix = np.array([[-sin_lon, cos_lon, 0],
                                 [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
                                 [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat]])
    # Apply rotation matrix to get ENU coordinates
    enu = np.dot(rotation_matrix, np.array([dx, dy, dz]))
    return enu

def ecef_to_gps(ecef):
    x, y, z = ecef
    a = 6378137.0
    finv = 298.257223563
    f = 1 / finv
    e2 = 1 - (1 - f) * (1 - f)
    lon = np.arctan2(y, x)
    p = np.sqrt(x**2 + y**2)
    lat = np.arctan2(z, p * (1 - e2))
    alt = 0.0
    prev_lat = 2 * np.pi
    while abs(lat - prev_lat) > 1e-9:
        prev_lat = lat
        v = a / np.sqrt(1 - e2 * np.sin(lat)**2)
        new_lat = np.arctan2(z + e2 * v * np.sin(lat), p)
        lat = new_lat
        alt = p / np.cos(lat) - v
    lat = lat * 180.0 / np.pi
    lon = lon * 180.0 / np.pi
    return lat, lon, alt

def enu_to_gps(enu, datum):
    # Retrieve datum ECEF coordinates
    datum_ecef = np.array(gps_to_ecef(datum))
    # Get observer's geodetic coordinates
    observer_lat = np.arctan2(datum_ecef[2], np.sqrt(datum_ecef[0]**2 + datum_ecef[1]**2))
    observer_lon = np.arctan2(datum_ecef[1], datum_ecef[0])
    # Calculate rotation matrix elements
    sin_lon = np.sin(observer_lon)
    cos_lon = np.cos(observer_lon)
    sin_lat = np.sin(observer_lat)
    cos_lat = np.cos(observer_lat)
    # Calculate inverse rotation matrix
    inv_rotation_matrix = np.array([[-sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon],
                                    [cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon],
                                    [0, cos_lat, sin_lat]])
    # Inverse rotate the ENU coordinates to get differences in ECEF coordinates
    ecef_diff = np.dot(inv_rotation_matrix, enu)
    # Add the differences to datum ECEF coordinates to get fix ECEF coordinates
    fix_ecef = datum_ecef + ecef_diff
    # Convert fix ECEF coordinates back to GPS coordinates
    return ecef_to_gps(fix_ecef)

class MyNode(Node):
    def __init__(self, args):
        super().__init__("cord_convert")
        self.get_logger().info('STARTING COORDINATE CONVERTER')

        self.gps2ecef_server = self.create_service(Gps2Ecef, '/fix/convert/gps2ecef', self.gps2ecef)
        self.ecef2gps_server = self.create_service(Ecef2Gps, '/fix/convert/ecef2gps', self.ecef2gps)

        self.ecef2enu_srver = self.create_service(Ecef2Enu, '/fix/convert/ecef2enu', self.ecef2enu)
        self.enu2ecef_server = self.create_service(Enu2Ecef, '/fix/convert/enu2ecef', self.enu2ecef)

        self.gps2enu_server = self.create_service(Gps2Enu, '/fix/convert/gps2enu', self.gps2enu)
        self.enu2gps_server = self.create_service(Enu2Gps, '/fix/convert/enu2gps', self.enu2gps) 

    def gps2ecef(self, request, response):
        self.get_logger().info('CONVERTING GPS TO ECEF')
        ecef_msg = Pose()
        lat, lon, alt = request.gps.latitude, request.gps.longitude, request.gps.altitude
        x, y, z = gps_to_ecef((lat, lon, alt))
        ecef_msg.position.x = x
        ecef_msg.position.y = y
        ecef_msg.position.z = z
        response.message = f"SUCCESSFULLY CONVERTED GPS TO ECEF"
        response.ecef = ecef_msg
        return response

    def ecef2gps(self, request, response):
        self.get_logger().info('CONVERTING ECEF TO GPS')
        gps_msg = NavSatFix()
        x, y, z = request.ecef.position.x, request.ecef.position.y, request.ecef.position.z
        lat, lon, alt = ecef_to_gps((x, y, z))
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        response.message = f"SUCCESSFULLY CONVERTED ECEF TO GPS"
        response.gps = gps_msg
        return response

    def ecef2enu(self, request, response):
        self.get_logger().info('CONVERTING ECEF TO ENU')
        self.get_logger().info('STILL NOT IMPLEMENTED')

    def enu2ecef(self, request, response):
        self.get_logger().info('CONVERTING ENU TO ECEF')
        self.get_logger().info('STILL NOT IMPLEMENTED')

    def gps2enu(self, request, response):
        self.get_logger().info('CONVERTING GPS TO ENU')
        gps = request.gps
        datum = request.datum
        enu = gps_to_enu((gps.latitude, gps.longitude, gps.altitude), (datum.latitude, datum.longitude, datum.altitude))
        enu_msg = Pose()
        enu_msg.position.x = enu[0]
        enu_msg.position.y = enu[1]
        enu_msg.position.z = enu[2]
        response.message = f"SUCCESSFULLY CONVERTED GPS TO ENU"
        response.enu = enu_msg
        return response

    def enu2gps(self, request, response):
        self.get_logger().info('CONVERTING ENU TO GPS')
        enu = request.enu
        datum = request.datum
        gps = enu_to_gps((enu.position.x, enu.position.y, enu.position.z), (datum.latitude, datum.longitude, datum.altitude))
        gps_msg = NavSatFix()
        gps_msg.latitude = gps[0]
        gps_msg.longitude = gps[1]
        gps_msg.altitude = gps[2]
        response.message = f"SUCCESSFULLY CONVERTED ENU TO GPS"
        response.gps = gps_msg
        return response


def main(args=None):
    rclpy.init(args=args)
    navfix = MyNode(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()