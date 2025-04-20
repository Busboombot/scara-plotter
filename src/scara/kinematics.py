import numpy as np

def _forward(l1, l2, T1, T2, as_degrees=False):
    """
    Calculate the forward kinematics of a 2D robotic arm with two links.
    
    Parameters:
    l1 (float): Length of the first link
    l2 (float): Length of the second link
    T1 (float): Angle of the first joint in radians
    T2 (float): Angle of the second joint in radians
    
    Returns:
    tuple: (X, Y) coordinates of the end
    """
    
    if as_degrees:
        T1 = np.radians(T1)
        T2 = np.radians(T2)
        print("converted to radians", T1, T2)

        
    X = l1 * np.cos(T1) - l2 * np.cos(T1 - T2 ) 
    Y = l1 * np.sin(T1) - l2 * np.sin(T1 - T2 ) 

    return np.round(X, 2), np.round(Y, 2)

# Inverse kinematics
def _inverse(l1, l2, x, y, as_degrees=False):
    """
    Calculate the inverse kinematics of a 2D robotic arm with two links,
    matching the forward kinematics:
               
        X = l1 * np.cos(T1) - l2 * np.cos(T1 - T2) 
        Y = l1 * np.sin(T1) - l2 * np.sin(T1 - T2) 
    
    Returns:
    tuple: (T1, T2) angles of the joints in radians
    """
    # Calculate the angles using the law of cosines
    
    # Compute the squared distance to the target
    r_squared = x**2 + y**2

    # Compute angle T2 (elbow-down configuration: positive angle)
    cos_t2 = (r_squared - l1**2 - l2**2) / (2 * l1 * l2)
    if np.abs(cos_t2) > 1:
        raise ValueError("Target is unreachable")

    t2 = np.arccos(cos_t2)  # elbow-down

    # Compute angle T1
    k1 = l1 + l2 * np.cos(t2)
    k2 = l2 * np.sin(t2)
    t1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    t2 = np.pi - t2 

    if as_degrees:
        t1 = np.degrees(t1)
        t2 = np.degrees(t2)

    return t1, t2
    
def mk_kine_pair(l1, l2):
    """
    Create forward and inverse kinematics functions with fixed link lengths.

    Parameters:
    l1 (float): Length of the first link
    l2 (float): Length of the second link

    Returns:
    tuple: (forward, inverse) functions
    """
    def forward(T1, T2, as_degrees=False):
        return _forward(l1, l2, T1, T2, as_degrees)

    def inverse(X, Y, as_degrees=False):
        return _inverse(l1, l2, X, Y, as_degrees=as_degrees)

    return forward, inverse


