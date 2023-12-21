"""
A collection of mathematical functions that accept and return tuples.
Module started on April 27, 2013.
Last modified by Dave Pigram on 02 January 2016.
"""

try:
    import math
    from math import sqrt
    # import Rhino.Geometry as rg
except ImportError as error:
    print "You don't have module :0 installed".format(error.message[16:])

__author__ = ['Dave Pigram', ]
__copyright__ = 'Copyright 2016, supermanoeuvre Pty Ltd'
__version__ = '0.1'
__email__ = 'dave@supermanoeuvre.com'
__status__ = 'Development'
__date__ = 'Jan 01, 2016'

global_tolerance = 0.002


def average(vals):
    """ Calculate the average of the provided values.  Uses the builtin sum

    Parameters:
        vals (iterable): An iterable collection of numbers (float or int)

    Returns:
        average (float): The average of the values in the list as a float
    """
    return sum(vals) / float(len(vals))


def limit_range(val, min_val, max_val):
    """Ensures a value is within a number range

    Args:
        val (float):
        min_val (float): Minimum value
        max_val (float): Maximum value

    Returns:
        float: A value within the number range. This number may be clipped to the upper and lower value specified
    """
    if val < min_val:
        return min_val
    elif val > max_val:
        return max_val
    else:
        return val


def CCW_2D(pt0, pt1, pt2):
    """Test if the turn formed by A, B, and C is ccw

    Parameters:
        pt0 (tuple): The x,y,z coordinates of point 1.
        pt1 (tuple): The x,y,z coordinates of point 2.
        pt2 (tuple): The x,y,z coordinates of point 3.

    Returns:
        CCW (boolean): True if the turn is counter-clockwise, False if it is clockwise.
    """
    return (pt1[0] - pt0[0]) * (pt2[1] - pt0[1]) > (pt1[1] - pt0[1]) * (pt2[0] - pt0[0])


def centroid(points):
    """Compute the centroid of the sequence of points.

    Note:
        Duplicate points are **NOT** removed. If there are duplicates in the
        sequence, they should be there intentionally.

    Parameter:
        points (sequence): A sequence of XYZ coordinates.

    Returns:
        tuple: XYZ coordinates of the centroid.
    """
    p = (len(points))
    return tuple(axis / p for axis in map(sum, zip(*points)))


def closest_point_index(pt0, pts, return_dist=False):
    """
    i, dist_sq = sorted({i : (pt0[0] - pt[0]) ** 2 +
                             (pt0[1] - pt[01]) ** 2 +
                             (pt0[2] - pt[2]) ** 2 for i, pt in enumerate(pts)})[0]
    """
    lookup = {(pt0[0] - pt[0]) ** 2 + (pt0[1] - pt[01]) ** 2 + (pt0[2] - pt[2]) ** 2: i for i, pt in enumerate(pts)}
    key = sorted(lookup)[0]
    if return_dist is not False:
        return lookup[key], sqrt(key)
    else:
        return lookup[key]


def min_distance(pt0, pts):
    """ Calculate the minimum distance from the test point to one of the points in a provided iterable collection.

    Parameters:
        pt0 (tuple): x,y,z coordinate of the focus point.
        pts (iterable): Iterable collection of x,y,z coordinates of points.
        return_dist (boolean):

    Returns:
        distance (float): The distance to the closest point.
    """
    return sqrt(sorted([(pt0[0] - pt[0]) ** 2 + (pt0[1] - pt[1]) ** 2 + (pt0[2] - pt[2]) ** 2 for pt in pts])[0])


def distance(pt0, pt1):
    """Calculate the distance between two points.

    Parameters:
        pt0 (tuple): x,y,z coordinate of the first point.
        pt1 (tuple): x,y,z coordinate of the second point.

    Return:
        distance (): The distance between the points.
    """
    return sqrt((pt1[0] - pt0[0]) ** 2 + (pt1[1] - pt0[1]) ** 2 + (pt1[2] - pt0[2]) ** 2)


def distance_squared(pt0, pt1):
    """Calculate the distance squared between two points.
    Useful for optimising loops where the point with the minimum squared distance also has the minimum distance.
    This the costly square-root function does not need to be called for all points.

    Parameters:
        pt0 (tuple): x,y,z coordinate of the first point.
        pt1 (tuple): x,y,z coordinate of the second point.

    Return:
        distance (): The distance between the points.
    """
    return (pt1[0] - pt0[0]) ** 2 + (pt1[1] - pt0[1]) ** 2 + (pt1[2] - pt0[2]) ** 2


def distance_weighted_value(pt, points, values):

    distances = [distance(pt, points[i]) for i in range(len(points))]
    total_distance = sum(distances)
    return sum(values[i] * ((total_distance - distance)/total_distance) for i, distance in enumerate(distances)) / (len(points) - 1)


def law_cosines_angle(a, b, c):
    """Use law of cosines to solve the included angle between side a and side b.

    Parameters:
        a (): length of side a
        b (): length of side b
        c (): length of side c

    Returns:
        angle (): Angle in degrees
    """
    return math.degrees(math.acos(((a ** 2) + (b ** 2) - (c ** 2)) / (2 * a * b)))


def law_cosines_side(a, b, C):
    """Use law of cosines to solve the length of side c.

    Parameters:
        a: length of side a.
        b: length of side b.
        C: included angle between side a and side b.

    Returns:
        length: length of side C
    """
    return sqrt(a ** 2 + b ** 2 - 2 * a * b * math.cos(math.radians(C)))


def centre_3_point_circle(pt0, pt1, pt2):
    """
    Method translated into python from Nick Pisca's example posted here

    Parameters:
        pt0 (tuple): The x,y,z coordinates of the first point.
        pt1 (tuple): The x,y,z coordinates of the second point.
        pt2 (tuple): The x,y,z coordinates of the third point.

    Returns:
        centre (tuple): The centre of the circle that circumscribes the three input points as a tuple (x.y.z).
    """
    #static double[] GetCen(pt0[0], pt0[1], pt0[2], pt1[0], pt1[2], pt1[2], pt2[0], pt2[1], pt2[2])
    r1 = pt0[0] - pt2[0]
    r2 = pt0[1] - pt2[1]
    r3 = pt0[2] - pt2[2]
    q1 = pt1[0] - pt2[0]
    q2 = pt1[1] - pt2[1]
    q3 = pt1[2] - pt2[2]
    t1 = r2 * q1 * r3**2.0 * q2   #NOTE:  the negative is NOT included
    t2 = q1 * r3**2.0 * q2**2.0
    t3 = r3 * q3 * r1 * q2**2.0
    t4 = r3 * q3 * r1 * q1**2.0
    t5 = r3 * q3 * r1**2.0 * q1
    t6 = r3 * r2**2.0 * q3 * q1
    t7 = q3**2.0 * r1 * r3**2.0
    t8 = q1 * r2**3.0 * q2
    t9 = q1 * r2**2.0 * q2**2.0
    t10 = q1 * r2**2.0 * q3**2.0
    t11 = r2 * q1**2.0 * r1 * q2
    t12 = r2 * q1 * r1**2.0 * q2
    t13 = q1 * r3**2.0 * q3**2.0
    t14 = r2**2.0 * q1**3.0
    t15 = q1**3.0 * r3**2.0
    t16 = r3 * q3**3.0 * r1
    t17 = r1**3.0 * q3**2.0
    t18 = q3 * q1 * r3**3.0
    t19 = q2**3.0 * r1 * r2
    t20 = q3**2.0 * r1 * r2 * q2
    t21 = r1 * r2**2.0 * q3**2.0
    t22 = r3**2.0 * r1 * q2**2.0
    t23 = q2**2.0 * r1 * r2**2.0
    t24 = r1**3.0 * q2**2.0
    numx = (-1 * t1) + t2 - t3 - t4 - t5 - t6 + t7 - t8 + t9 + t10 - t11 - t12 + t13 + t14 + t15 - t16 + t17 - t18 - t19 - t20 + t21 + t22 + t23 + t24
    s1 = r2 * r1 * q1**3.0
    s2 = q1**2.0 * r2**3.0
    s3 = q1**2.0 * r2 * r1**2.0
    s4 = q1**2.0 * r2 * q3 * r3
    s5 = q1**2.0 * r2 * r3**2.0
    s6 = q1**2.0 * r3**2.0 * q2
    s7 = q1**2.0 * r1**2.0 * q2
    s8 = q1 * r2**2.0 * q2 * r1
    s9 = q1 * r2 * q2**2.0 * r1
    s10 = q1 * r2 * q3**2.0 * r1
    s11 = q1 * r1**3.0 * q2
    s12 = q1 * r3**2.0 * r1 * q2
    s13 = q3**2.0 * r2**3.0
    s14 = r3 * q3 * q2 * r2**2.0
    s15 = r3**2.0 * r2 * q3**2.0
    s16 = r2 * q3**2.0 * r1**2.0
    s17 = r2 * r3 * q3 * q2**2.0
    s18 = r2 * r3 * q3**3.0
    s19 = q2**3.0 * r1**2.0
    s20 = q3 * r3**3.0 * q2
    s21 = r3 * q3 * r1**2.0 * q2
    s22 = q3**2.0 * r1**2.0 * q2
    s23 = r3**2.0 * q2**3.0
    s24 = r3**2.0 * q2 * q3**2.0
    numy = s1 - s2 - s3 + s4 - s5 - s6 - s7 + s8 + s9 + s10 + s11 + s12 - s13 + s14 - s15 - s16 + s17 + s18 - s19 + s20 + s21 - s22 - s23 - s24
    v1 = q3 * r1**2.0 * q2**2.0      #negative not put on this variable
    v2 = q3 * r1**2.0 * q1**2.0
    v3 = r3**3.0 * q2**2.0
    v4 = r2**2.0 * q3**3.0
    v5 = q1**2.0 * r3**3.0
    v6 = q3 * r1**3.0 * q1
    v7 = q1**2.0 * r3 * r1**2.0
    v8 = r3 * q2**2.0 * r1**2.0
    v9 = q1**3.0 * r3 * r1
    v10 = r3 * q2 * r2 * q3**2.0
    v11 = r2 * q3 * r3**2.0 * q2
    v12 = q1 * r3 * r1 * q3**2.0
    v13 = q3 * r1 * q1 * r3**2.0
    v14 = q3**3.0 * r1**2.0
    v15 = r3 * q2**3.0 * r2
    v16 = r2**2.0 * q3 * q1 * r1
    v17 = r2**2.0 * q3 * q1**2.0
    v18 = r2**2.0 * q3 * q2**2.0
    v19 = q1**2.0 * r3 * r2**2.0
    v20 = q1 * r3 * r1 * q2**2.0
    v21 = r3 * q2**2.0 * r2**2.0
    v22 = r2 * q3 * r1**2.0 * q2
    v23 = r2**3.0 * q3 * q2
    v24 = r3 * q2 * r2 * q1**2.0
    numz = (-1 * v1) - v2 - v3 - v4 - v5 + v6 - v7 - v8 + v9 + v10 + v11 + v12 + v13 - v14 + v15 + v16 - v17 - v18 - v19 + v20 - v21 + v22 + v23 + v24
    n1 = q2**2.0 * r1**2.0
    n2 = q3**2.0 * r1**2.0
    n3 = r2**2.0 * q3**2.0
    n4 = q1**2.0 * r3**2.0
    n5 = r3**2.0 * q2**2.0
    n6 = q1**2.0 * r2**2.0
    n7 = 2 * r3 * q2 * r2 * q3
    n8 = 2 * q3 * r1 * q1 * r3
    n9 = 2 * q1 * r2 * r1 * q2
    den = n1 + n2 + n3 + n4 + n5 + n6 - n7 - n8 - n9
    if (den != 0.0):
        x = (0.5 * (numx / den)) + pt2[0]
        y = ((-0.5) * (numy / den)) + pt2[1]
        z = ((-0.5) * (numz / den)) + pt2[2]
        return (x, y, z)
    else:
        return None


def radius_3_point_circle(pt0, pt1, pt2):
    centre = centre_3_point_circle(pt0, pt1, pt2)
    if centre:
        return distance(pt0, centre)
    else:
        return None

# _______________________________________________________________________________________________________________________
# Point and Vector Operations using tuples

def angle_between_vecs(v0, v1, axis=(0, 0, 1)):
    """Calculate the angle in degrees between two vectors.

    Parameters:
        v0 (tuple): The first vector (x,y,z).
        v1 (tuple): The second vector (x,y,z).

    Returns:
        Angle (): The angle in degrees.
    """
    unit_vec_0 = vec_unit(v0)
    unit_vec_1 = vec_unit(v1)
    dot = vec_dot(unit_vec_0, unit_vec_1)
    angle = math.degrees(math.acos(dot))
    # Check if the angle should be positive or negative
    pos_rot_v0 = vec_rotate(unit_vec_0, (0, 0, 0), axis, angle)
    # diffPosRot = (unit_vec_1.X - vec1.X, unit_vec_1.Y - vec1.Y, unit_vec_1.Z - vec1.Z)
    diff_pos_rot = vec_subtract(unit_vec_1, pos_rot_v0)
    # vec1.Rotate(-2 * angleRad, axis)
    neg_rot_v0 = vec_rotate(unit_vec_0, (0, 0, 0), axis, -angle)
    diff_neg_rot = vec_subtract(unit_vec_1, neg_rot_v0)
    # diffNegRot = (unit_vec_1.X - vec1.X, unit_vec_1.Y - vec1.Y, unit_vec_1.Z - vec1.Z)
    if vec_length(diff_pos_rot) > vec_length(diff_neg_rot):
        angle *= -1
    return angle


def angle_between_vecs_in_plane(v0, v1, plane_normal=(0, 0, 1)):
    """Calculate the angle in degrees between two vectors with both projected onto a plane defined by its normal.

    Parameters:
        v0 (tuple): The first vector (x,y,z).
        v1 (tuple): The second vector (x,y,z).
        plane_normal (tuple): The normal of the plane that the other vectors will be projected onto (x,y,z).

    Returns:
        Angle (): The angle in degrees.
    """
    projected_v0 = project_vector_onto_plane(v0, plane_normal)
    projected_v1 = project_vector_onto_plane(v1, plane_normal)
    return angle_between_vecs(projected_v0, projected_v1, plane_normal)


def angle_with_negative_lines(line_00, line_01, pos_vec):
    """Calculate the angle in degrees between two lines using a reference vector to determine positive direction.

    Parameters:
        line_00 (tuple): The first line as a tuple of tuples ((x,y,z),(x,y,z)).
        line_01 (tuple): The second line as a tuple of tuples ((x,y,z),(x,y,z)).
        pos_vec (tuple): A vector (x,y,z) that determines the positive direction (right-hand rule).

    Returns:
        Angle (): The angle in degrees.
    """

    unit_v0 = vec_unit(vec_subtract(line_00[1], line_00[0]))
    unit_v1 = vec_unit(vec_subtract(line_01[1], line_01[0]))
    unit_pos_vec = vec_unit(pos_vec)

    vecCrossProduct = vec_cross(unit_v0, unit_v1)
    if vecCrossProduct:
        a = distance((0, 0, 0), unit_v0)
        b = distance((0, 0, 0), unit_v1)
        c = distance(unit_v0, unit_v1)
        angle = law_cosines_angle(a, b, c)
        # To test the sign of the angle, see if the pos_vec matches the direction of that of the cross product
        if distance(unit_pos_vec, vecCrossProduct) > 1: angle = -angle
    else:
        # The vectors are colinear, find out if they are cooincident (0 degrees) or opposite (180 degrees)
        if distance(unit_v0, unit_v1) > global_tolerance:
            angle = 180
        else:
            angle = 0
    return angle


def angle_with_negative_vecs(v0, v1, pos_vec):
    """Calculate the angle in degrees between two vectors using a reference vector to determine positive direction.

    Parameters:
        v0 (tuple): The first vector (x,y,z).
        v1 (tuple): The second vector (x,y,z).
        pos_vec (tuple): A vector (x,y,z) that determines the positive direction (right-hand rule).

    Returns:
        Angle (): The angle in degrees.
    """

    unit_v0 = vec_unit(v0)
    unit_v1 = vec_unit(v1)
    unit_pos_vec = vec_unit(pos_vec)

    v_cross = vec_cross(unit_v0, unit_v1)
    if v_cross:
        a = distance((0, 0, 0), unit_v0)
        b = distance((0, 0, 0), unit_v1)
        c = distance(unit_v0, unit_v1)
        angle = law_cosines_angle(a, b, c)
        # To test the sign of the angle, see if the pos_vec matches the direction of that of the plane from points
        if distance(unit_pos_vec, v_cross) > distance(vec_reverse(unit_pos_vec), v_cross):
            angle = -angle
    else:
        # The vectors are colinear, find out if they are cooincident (0 degrees) or opposite (180 degrees)
        if distance(unit_v0, unit_v1) > global_tolerance:
            angle = 180
        else:
            angle = 0
    return angle


def midpoint(pt0, pt1):
    """Calculate the midpoint between two points.

    Parameters:
        pt0 (tuple): The x,y,z coordinates of the first point.
        pt1 (tuple): The x,y,z coordinates of the second point.

    Returns:
        midpoint (tuple): The x,y,z coordinates of the midpoint.
    """
    return ((pt0[0] + pt1[0]) / 2, (pt0[1] + pt1[1]) / 2, (pt0[2] + pt1[2]) / 2)


def chamfer_corner_distances(pt0, pt1, pt2, distance0, distance1):
    """ Calculate the ocation of points that would define a chamfer across a corner.
    The chamfer is defined by the distances along the edges from the corner.

    Parameters:
        pt0 (tuple): The coordinates of a point in the direction of the edge before the corner as a tuple (x,y,z).
        pt1 (tuple): The coordinates of the corner point location as a tuple (x,y,z).
        pt2 (tuple): The coordinates of a point in the direction of the edge after the corner as a tuple (x,y,z).
        distance0 (float): The desired distance ahead of the corner that the chamfer should begin.
        distance1 (float): The desired distance after of the corner that the chamfer should end.

    Returns:
        chamfer_pt_0 (tuple): The coordinates of the point at the beginning of the chamfer as a tuple (x,y,z).
        chamfer_pt_1 (tuple): The coordinates of the point at the end of the chamfer as a tuple (x,y,z).
    """
    v0 = vec_subtract(pt0, pt1)
    v1 = vec_subtract(pt2, pt1)
    v0 = vec_force_length(v0, distance0)
    v1 = vec_force_length(v1, distance1)
    return vec_add(pt1, v0), vec_add(pt1, v1)


def chamfer_corner_length(pt0, pt1, pt2, length):
    """ Calculate the ocation of points that would define a chamfer across a corner.
    The chamfer is defined by the distances along the edges from the corner.

    Parameters:
        pt0 (tuple): The coordinates of a point in the direction of the edge before the corner as a tuple (x,y,z).
        pt1 (tuple): The coordinates of the corner point location as a tuple (x,y,z).
        pt2 (tuple): The coordinates of a point in the direction of the edge after the corner as a tuple (x,y,z).
        length (float):  The desired resultant length of the chamfer line.

    Returns:
        chamfer_pt_0 (tuple): The coordinates of the point at the beginning of the chamfer as a tuple (x,y,z).
        chamfer_pt_1 (tuple): The coordinates of the point at the end of the chamfer as a tuple (x,y,z).
    """
    v0 = vec_subtract(pt0, pt1)
    v1 = vec_subtract(pt2, pt1)
    angle = angle_between_vecs(v0, v1)
    chamfer_height = (length / 2) / math.tan(math.radians(angle / 2))
    distance = law_cosines_side(chamfer_height, length / 2, 90)
    v0 = vec_force_length(v0, distance)
    v1 = vec_force_length(v1, distance)
    return vec_add(pt1, v0), vec_add(pt1, v1)


def line_closest_point_unbounded(v0, v1, pt):
    """

    Args:
        v0 (tuple):
        v1 (tuple):
        pt (tuple):

    Returns:
        closest_point (tuple): The closest point to the line. Points outside the bounds are permitted.
    """
    unit_line_vec = vec_unit(vec_subtract(v1, v0))
    v0_to_pt = vec_subtract(pt, v0)

    scalar = vec_dot(v0_to_pt, unit_line_vec)
    translation_vec = vec_scale(unit_line_vec, scalar)
    closest_point = vec_add(v0, translation_vec)
    return closest_point


def line_closest_point_bounded(line_pt0, line_pt1, pt):
    """
    
    Args:
        v0 (tuple):
        v1 (tuple):
        pt (tuple):

    Returns:
        closest_point (tuple): The closest point to the line. Points outside the bounds will return the closest end point.
    """
    # Localise external function calls
    closest_point = line_closest_point_unbounded(line_pt0, line_pt1, pt)
    # Check to see if closest point lies within
    # the domain of the curve (actually lies on the line!)
    # If not return start point or end point - whichever is closest!
    line_length = distance(line_pt0, line_pt1)
    dist_line_pt0 = distance(line_pt0, closest_point)
    dist_line_pt1 = distance(line_pt1, closest_point)
    if (dist_line_pt0 < line_length) and (dist_line_pt1 < line_length):
        return closest_point
    elif (dist_line_pt0 < dist_line_pt1):
        return line_pt0
    else:
        return line_pt1


def lines_closest_point_unbounded(lines, pt):
    return sorted([line_closest_point_unbounded(line_pt0, line_pt1, pt) for line_pt0, line_pt1 in lines],
           key=lambda closest_point : distance(closest_point, pt))[0]


def lines_closest_point_bounded(lines, pt):
    return sorted([line_closest_point_bounded(line_pt0, line_pt1, pt) for line_pt0, line_pt1 in lines],
           key=lambda closest_point : distance(closest_point, pt))[0]


def line_divide(line, divisions):
    """ Calculate the points that equally divide between the start and end points of a line by the specified number of divisions.

    Parameters:
        line (iterable): The start and end points of the line as tuples (x,y,z)
        divisions (integer): The number of division points

    Returns:
        division_points (list): The coordinates of the division pounts as tuples (x,y,z)
    """
    vec = vec_divide(vec_subtract(line[1], line[0]), divisions)
    return [vec_add(line[0], vec_scale(vec, i)) for i in xrange(divisions + 1)]


def offset_corner(pt0, pt1, pt2, offsets=(1, 1), extra_returns=False):
    """Find the offset inside corner point.  Edge pt0-pt1 is offset By OFFSET[0] edge Pt1-Pt2 is offset by OFFSET[1]

    Parameters:
        pt0 (tuple): The x,y,z coordinates of a point along the first edge.
        pt1 (tuple): The x,y,z coordinates of the original corner.
        pt2 (tuple): The x,y,z coordinates of a point along the second edge.

    Returns:
        point (tuple): The offset inside corner point.

    """
    # create planes for each edge where z is the offset vector
    plane1XVec = vec_subtract(pt0, pt1)
    plane2XVec = vec_subtract(pt2, pt1)
    cornerNormal = vec_cross(plane1XVec, plane2XVec)
    plane1ZVec = vec_cross(cornerNormal, plane1XVec)
    plane2ZVec = vec_cross(cornerNormal, plane2XVec)
    plane1 = (pt1, vec_unit(plane1XVec), vec_unit(vec_cross(plane1XVec, plane1ZVec)),
              vec_unit(plane1ZVec))
    plane2 = (pt1, vec_unit(plane2XVec), vec_unit(vec_cross(plane2XVec, plane2ZVec)),
              vec_unit(plane2ZVec))
    # make sure each plane's z points into the corner
    if vec_dot(plane1ZVec, plane2XVec) < 0: plane1 = plane_reverse(plane1)
    if vec_dot(plane2ZVec, plane1XVec) < 0: plane2 = plane_reverse(plane2)
    # create the offset vectors
    offsetVec1 = vec_force_length(plane1[3], offsets[0])
    offsetVec2 = vec_force_length(plane2[3], offsets[1])
    offsetLine1 = (vec_add(pt0, offsetVec1), vec_add(pt1, offsetVec1))
    offsetPlane2 = (
        vec_add(plane2[0], offsetVec2), plane2[1], plane2[2], plane2[3])  # offset plane by moving its origin
    intersection, intType = line_plane_intersection(offsetLine1, offsetPlane2)
    if not extra_returns:
        if intType == 0:
            return intersection
        else:
            return None
    else:
        # Return also the intersection of the offset planes with the original lines (to be able to make certain offset meshes)
        offsetLine2 = (vec_add(pt1, offsetVec2), vec_add(pt2, offsetVec2))
        intersection1, int1Type = line_plane_intersection(offsetLine2, plane1)
        intersection2, int2Type = line_plane_intersection(offsetLine1, plane2)
        if intType == 0 and int1Type == 0 and int2Type == 0:
            return (intersection, intersection1, intersection2)


def offset_corner_3d(corner, pt0, pt1, pt2, offsets=(1, 1, 1)):
    """Find the offset inside corner point.  Edge pt0-pt1 is offset By OFFSET[0] edge Pt1-Pt2 is offset by OFFSET[1]

    Parameters:
        pt0 (tuple): The x,y,z coordinates of a point along the first edge.
        pt1 (tuple): The x,y,z coordinates of the original corner.
        pt2 (tuple): The x,y,z coordinates of a point along the second edge.
        offsets (iterable): the offset value for each plane ordered: (corner, 0, 1) (corner, 1, 2) (corner, 2, 0) - negative for

    Returns:
        point (tuple): The offset inside corner point.
    """
    # create the initial planes
    plane_0 = plane_from_points(corner, pt0, pt1)
    plane_1 = plane_from_points(corner, pt1, pt2)
    plane_2 = plane_from_points(corner, pt2, pt0)

    # offset the planes
    plane_0 = plane_offset(plane_0, offsets[0])
    plane_1 = plane_offset(plane_1, offsets[1])
    plane_2 = plane_offset(plane_2, offsets[2])

    # Intersect the first two planes to get a line
    line = plane_plane_intersection(plane_0, plane_1)[0]

    # Intersect the line with the plane
    return line_plane_intersection(line, plane_2)[0]


def get_fillet_data(pt0, pt1, pt2, radius):
    """Find the centre of a fillet; the points on the two lines where the fillet starts and ends and the axis of the fillet.

    Parameters:
        pt0 (tuple): The x,y,z coordinates of a point along the first edge.
        pt1 (tuple): The x,y,z coordinates of the original corner.
        pt2 (tuple): The x,y,z coordinates of a point along the second edge.
        radius (float): The fillet radius.

    Returns:
        point (tuple): The centre of the fillet.
        point (tuple): The point on the first line where the fillet arc starts.
        point (tuple): The point on the second line where the fillet arc end.
        axis (tuple): The x,y,z values of the axis of the fillet arc.

    """
    # create planes for each edge where z is the offset vector
    plane1XVec = vec_subtract(pt0, pt1)
    plane2XVec = vec_subtract(pt2, pt1)
    cornerNormal = vec_cross(plane1XVec, plane2XVec)
    plane1ZVec = vec_cross(cornerNormal, plane1XVec)
    plane2ZVec = vec_cross(cornerNormal, plane2XVec)
    plane1 = (pt1, vec_unit(plane1XVec), vec_unit(vec_cross(plane1XVec, plane1ZVec)),
              vec_unit(plane1ZVec))
    plane2 = (pt1, vec_unit(plane2XVec), vec_unit(vec_cross(plane2XVec, plane2ZVec)),
              vec_unit(plane2ZVec))
    # make sure each plane's z points into the corner
    if vec_dot(plane1ZVec, plane2XVec) < 0: plane1 = plane_reverse(plane1)
    if vec_dot(plane2ZVec, plane1XVec) < 0: plane2 = plane_reverse(plane2)
    # create the offset vectors
    offsetVec1 = vec_force_length(plane1[3], radius)
    offsetVec2 = vec_force_length(plane2[3], radius)
    offsetLine1 = (vec_add(pt0, offsetVec1), vec_add(pt1, offsetVec1))
    offsetLine2 = (vec_add(pt1, offsetVec2), vec_add(pt2, offsetVec2))
    offsetPlane2 = (
        vec_add(plane2[0], offsetVec2), plane2[1], plane2[2], plane2[3])  # offset plane by moving its origin
    centre_pt, intType = line_plane_intersection(offsetLine1, offsetPlane2)
    if intType != 0:
        return None
    else:
        # find the closest point on each line as the start and end of the fillet
        fillet_start = line_closest_point_unbounded(pt0, pt1, centre_pt)
        fillet_end = line_closest_point_unbounded(pt1, pt2, centre_pt)
        return centre_pt, fillet_start, fillet_end, cornerNormal


def project_point_towards(pt0, pt1, length=1):
    vec = vec_subtract(pt1, pt0)
    length = vec_length(vec)
    try:
        vec = vec_scale(vec, length / length)
        return vec_add(pt0, vec)
    except:
        return None


def project_vector_onto_plane(v0, plane_normal):
    unit_normal = vec_unit(plane_normal)
    dot = vec_dot(v0, unit_normal)
    projection_vec = vec_scale(unit_normal, dot)
    return vec_subtract(v0, projection_vec)


def points_average(pts):
    numPts = len(pts)
    x = 0
    y = 0
    z = 0
    for p0 in pts:
        x += p0[0]
        y += p0[1]
        z += p0[2]
    return (x / numPts, y / numPts, z / numPts)


def point_round_pos(pt, sanitise=True):
    if sanitise:
        if pt[0]**2 < 1e-9:
            pt[0] = 0.0
        if pt[1]**2 < 1e-9:
            pt[1] = 0.0
        if pt[2]**2 < 1e-9:
            pt[2] = 0.0
    return (round(pt[0]), round(pt[1]), round(pt[2]))


def q_round_coord(pt, resolution=0.1, sanitise=True):
    if sanitise:
        if pt[0]**2 < 1e-9:
            pt = (0.0, pt[1], pt[2])
        if pt[1]**2 < 1e-9:
            pt = (pt[0], 0.0, pt[2])
        if pt[2]**2 < 1e-9:
            pt = (pt[0], pt[1], 0.0)
    return tuple(resolution * round(pt[index] / float(resolution)) for index in range(3))


def q_round_neighbour_keys(pt, resolution=0.1):
    voxel_key = q_round_coord(pt, resolution, round=2)
    keys = []
    x, y, z = voxel_key
    for i in (-1,0,1):
        for j in (-1,0,1):
            for k in (-1,0,1):
                keys.append((x + k * resolution, y + j * resolution, z + i * resolution))
    return keys


def q_round_coord_decimal(COORD, RES):
    pass
    #from decimal import Decimal
    #return tuple(Decimal(COORD[_]).quantize(Decimal(str(RES))) for _ in range(3))


def ruling_grid(points_0, points_1, divisions):
    """ Calculate the points that form a divided ruling between the two point sequences.


    Args:
        points_0 (iterable): An ordered sequence of points that define the first curve/line.
        points_1 (iterable): An ordered sequence of points that define the second curve/line.
        divisions (integer): The number of

    Returns:
        points_grid (list): A list of points with (divisions + 1) x len(points_0) members.
    """
    points_grid = []
    for i in xrange(len(points_0)):
        points_grid.extend(line_divide((points_0[i], points_1[i]), divisions))
    return points_grid


def vector3D_to_tuple(VEC3D):
    return (VEC3D.X, VEC3D.Y, VEC3D.Z)


def vec_unit(v0):
    # accepts and returns a tuple
    vecLength = vec_length(v0)
    if vecLength == 0:
        return (0, 0, 0)
        print 'unit vector error'
    return (v0[0] / vecLength, v0[1] / vecLength, v0[2] / vecLength)


def vec_add(v0, v1):
    # accepts two tuples and returns a tuple
    return (v0[0] + v1[0], v0[1] + v1[1], v0[2] + v1[2])


def vec_average(v0, v1):
    # Calculate the average of two vectors.
    """

    Parameters:
        v0 (tuple): The first vector (x, y, z).
        v1 (tuple): The second vector (x, y, z).

    Returns:
        vec (tuple): The average 3d vector (x,y,z).
    """
    return ((v0[0] + v1[0]) / 2, (v0[1] + v1[1]) / 2, (v0[2] + v1[2]) / 2)


def vecs_average(vecs):
    # Calculate the average of a collection of vectors.
    """

    Parameters:
        vecs (iterable): An iterable collection of vectors as yuples (x, y, z).

    Returns:
        vec (tuple): The average 3d vector (x,y,z).
    """
    numVecs = len(vecs)
    """
    if numVecs == 1:
        return vecs[0]
    """
    x = 0
    y = 0
    z = 0
    for vec in vecs:
        x += vec[0]
        y += vec[1]
        z += vec[2]
    return (x / numVecs, y / numVecs, z / numVecs)


def vec_dot(v0, v1):
    """Calculate the dot product of two vectors.

    Parameters:
        v0 (tuple): The first vector (x,y,z).
        v1 (tuple): The second vector (x,y,z).

    Returns:
        dot (): The dot-product.
    """
    return v0[0]*v1[0] + v0[1]*v1[1] + v0[2]*v1[2]


def vec_cross(v0, v1):
    """Calculate the cross-product of two vectors.

    Parameters:
        v0 (tuple): The first vector (x,y,z).
        v1 (tuple): The second vector (x,y,z).

    Returns:
        Vector (tuple): The cross-product as a unit-vector.
    """
    return ((v0[1] * v1[2] - v0[2] * v1[1],
             v0[2] * v1[0] - v0[0] * v1[2],
             v0[0] * v1[1] - v0[1] * v1[0]))


def vec_divide(v0, scalar):
    # accepts and returns a tuple
    scalar = float(scalar) # To avoid the rounding that happens with integer division
    return (v0[0] / scalar, v0[1] / scalar, v0[2] / scalar)


def vec_force_length(v0, length):
    # accepts and returns a tuple
    if length == 0:
        return (0,0,0)
    elif vec_length(v0) > 0:
        unit = vec_unit(v0)
        return (unit[0] * length, unit[1] * length, unit[2] * length)
    else:
        raise ValueError("SuperMath.vec_force_length(): The vector cannot have a zero length")
        #return (0, 0, 0)


def vec_length(v0):
    """
    accepts and returns a tuple
    """
    # length equals square root of (x squared + y squared + z squared)
    try:
        return sqrt(v0[0] ** 2 + v0[1] ** 2 + v0[2] ** 2)
    except:
        return 0


def vec_lerp(v0, v1, unit_interval):
    """

    Arguments:
        v0 (tuple):
        v1 (tuple):
        unit_interval (): A number between 0.0 and 1.0 0.0 would return v0, 1.0 would return v1 and 0.5 would yield their midpoint

    Returns:
        point (tuple): The linear interpolated point (x, y, z).
    """
    """Returns the weighted average vector / point location as a tuple
    The unit_interval is requires as a number between 0 and 1 where 0.0 would return v0, 1.0 would return v1 and 0.5 would yield their midpoint
    """
    x = v0[0] + unit_interval * (v1[0] - v0[0])
    y = v0[1] + unit_interval * (v1[1] - v0[1])
    z = v0[2] + unit_interval * (v1[2] - v0[2])
    return (x, y, z)


def vec_subtract(v0, v1):
    # accepts two tuples and returns a tuple
    return (v0[0] - v1[0], v0[1] - v1[1], v0[2] - v1[2])


def vec_scale(v0, scalar):
    # accepts a tuple and a real or int and returns a tuple
    return (v0[0] * scalar, v0[1] * scalar, v0[2] * scalar)


def vec_reverse(v0):
    return (-v0[0], -v0[1], -v0[2])


def vec_rotate_rhino(v0, angle, axis=(0, 0, 1)):
    """
    Rotates a vector as tuple by an angle () in degrees
    """
    from Rhino.Geometry import Vector3d
    vec = Vector3d(v0[0], v0[1], v0[2])
    axis = Vector3d(axis[0], axis[1], axis[2])
    vec.Rotate(math.radians(angle), axis)
    return (vec.X, vec.Y, vec.Z)


def vec_rotate(v0, axis_pt0, axis_pt1, angle):
    """

    Parameters:
        v0 (tuple): The vector to be rotated.
        origin (tuple): The x,y,z coordinate of the origin of rotation.
        axis_vec (tuple): The direction of the
        angle (float): Rotation angle in degrees.

    Returns:
        rotated_vec (tuple):
    """
    # translate axis to origin
    pt_translated = vec_subtract(v0, axis_pt0)
    axis_translated = vec_unit(vec_subtract(axis_pt0, axis_pt1))

    # Matrix common factors
    angle_rad = math.radians(angle)

    c = math.cos(angle_rad)
    t = (1 - math.cos(angle_rad))
    s = math.sin(angle_rad)
    axis_x = axis_translated[0]
    axis_y = axis_translated[1]
    axis_z = axis_translated[2]

    # Transformation matrix
    d11 = t * pow(axis_x, 2) + c
    d12 = t * axis_x * axis_y - s * axis_z
    d13 = t * axis_x * axis_z + s * axis_y

    d21 = t * axis_x * axis_y + s * axis_z
    d22 = t * math.pow(axis_y, 2) + c
    d23 = t * axis_y * axis_z - s * axis_x

    d31 = t * axis_x * axis_z - s * axis_y
    d32 = t * axis_y * axis_z + s * axis_x
    d33 = t * math.pow(axis_z, 2) + c

    # Apply matrix transforms
    rx = d11 * pt_translated[0] + d12 * pt_translated[1] + d13 * pt_translated[2]
    ry = d21 * pt_translated[0] + d22 * pt_translated[1] + d23 * pt_translated[2]
    rz = d31 * pt_translated[0] + d32 * pt_translated[1] + d33 * pt_translated[2]

    return vec_add((rx, ry, rz), axis_pt0)


def vecs_sum(vecs):
    """
    Sum a list or tuple of vectors as tuples, returning a tuple (x,y,z)
    """
    numVecs = len(vecs)
    x = 0
    y = 0
    z = 0
    for vec in vecs:
        x += vec[0]
        y += vec[1]
        z += vec[2]

    return (x, y, z)


# ______________________________________________________________________________________________________________________
# Line Operations using tuples

def get_line_gradient_constant(pt0, pt1):
    """Return m (the gradient) and b (the constant) for a line y = mx + b. Lines must me on the XY plane.

    Parameters:
        pt0 (tuple): x,y,z coordinate of the first point on the line.
        pt1 (tuple): x,y,z coordinate of the second point on the line.

    Returns:
        m (): The gradient.
        b (): The constant.
    """
    m = (pt1[1] - pt0[1]) / (pt1[0] - pt0[0])  # rise/run
    b = pt0[1] - m * pt0[0]  # If y = mx + b, then b = y - mx
    return m, b


def line_line_intersection_2D(line_00, line_01):
    """Find the intersection of two lines.

    Parameters:
        line_00 (tuple): The first line as a tuple of tuples ((x,y,z),(x,y,z))
        line_01 (tuple): The second line as a tuple of tuples ((x,y,z),(x,y,z))

    Returns:
        intersection (tuple): The intersection point
    """
    m1, b1 = get_line_gradient_constant(line_00[0], line_00[1])
    m2, b2 = get_line_gradient_constant(line_01[0], line_01[1])
    x = (b2 - b1) / (m1 - m2)
    y = m1 * x + b1
    return (x, y, 0)


def line_plane_intersection(line, plane):
    """Find the intersection between a line and a plane.

    Parameters:
        line_00 (tuple): The line as a tuple of tuples ((x,y,z), (x,y,z))
        plane (tuple): The plane as a tuple of tuples (origin, x-axis, y-axis, z-axis)

    Returns:
        intersection (tuple): (x,y,z) if point, ((x,y,z), (x,y,z)) if line, NONE if no intersection.
        type (int): 0 for point, 1 for line, 2 for no intersection.
    """
    lineVec = vec_unit(vec_subtract(line[1], line[0]))
    numerator = vec_dot(vec_subtract(plane[0], line[0]), plane[3])
    denominator = vec_dot(lineVec, plane[3])
    if denominator != 0:
        # Intersection is a point, return the point coordinate
        return (vec_add(line[0], vec_force_length(lineVec, numerator / denominator)), 0)
    else:
        # line is parallel to the plane
        if numerator == 0:
            return (line, 1)  # The line intersects entirely (is on) the plane
        else:
            return (None, 2)  # There is no intersetion


# ______________________________________________________________________________________________________________________
# Plane Operations using tuples

def plane_from_points(origin, pt0, pt1):
    """Create a plane in the format: (origin, XVEC, YVEC, ZVEC) all as tuples.

    Parameters:
        origin (tuple): The x,y,z coordinates of the plane's origin.
        pt0 (tuple): The x,y,z coordinates of a point on the plane's x-axis.
        pt1 (tuple): The x,y,z coordinates of a point that defines the plan's positive y-axis.

    Returns:
        plane (tuple): The plane as a tuple of tuples (origin, x-axis, y-axis, z-axis).
    """
    xVec = vec_subtract(pt0, origin)
    vecEdge2 = vec_subtract(pt1, origin)
    zVec = vec_cross(xVec, vecEdge2)
    yVec = vec_cross(zVec, xVec)
    return (origin, vec_unit(xVec), vec_unit(yVec), vec_unit(zVec))


def plane_from_normal(normal, origin):
    """Create a plane in the format: (origin, XVEC, YVEC, ZVEC) all as tuples.

    Parameters:
        origin (tuple): The x,y,z coordinates of the plane's origin.
        normal (tuple): The x,y,z values of the plane's normal vector.

    Returns:
        plane (tuple): The plane as a tuple of tuples (origin, x-axis, y-axis, z-axis).
    """
    if vec_length(normal) == 0:
        raise ValueError("SuperMath.plane_from_normal(): The normal cannot have a zero length")
    # Use the y-axis as a default
    y_axis = (0, 1, 0)
    if vec_unit(y_axis) != vec_unit(normal):
        xVec = vec_cross(y_axis, normal)
        yVec = vec_cross(normal, xVec)
    else:
        x_axis = (1, 0, 0)
        yVec = vec_cross(normal, x_axis)
        xVec = vec_cross(yVec, normal)
    return (origin, vec_unit(xVec), vec_unit(yVec), vec_unit(normal))


def plane_plane_intersection(plane_0, plane_1):
    """Find the intersection between a line and a plane.

    Parameters:
        plane_0 (tuple): The first plane as a tuple of tuples (origin, x-axis, y-axis, z-axis)
        plane_1 (tuple): The second plane as a tuple of tuples (origin, x-axis, y-axis, z-axis)

    Returns:
        line_00 (tuple): The line as a tuple of tuples ((x,y,z), (x,y,z)), NONE if no intersection.
        type (int): 0 for line, 1 for plane, 2 for no intersection.
    """
    # Get the cross product of the normals
    cross_normals = vec_cross(plane_0[3], plane_1[3])
    if cross_normals is None:
        return (None, 2)  # The planes are parallel, there is no intersection
    # Find the point of intersection of the first plane's x-axis with the second plane
    int, type = line_plane_intersection((plane_0[0], vec_add(plane_0[0], plane_0[1])), plane_1)
    if type == 0:
        return ((int, vec_add(int, cross_normals)), 0)
    else:
        return (plane_0, 1)  # The planes intersect entirely


def plane_reverse(plane):
    """Flip the orientation of a plane.

    Parameters:
        plane (tuple): The first plane as a tuple of tuples (origin, x-axis, y-axis, z-axis)

    Returns:
        flipped_plane (tuple): The flipped plane as a tuple of tuples (origin, x-axis, y-axis, z-axis)
    """

    return (plane[0], plane[1], vec_reverse(plane[2]), vec_reverse(plane[3]))


def plane_offset(plane, offset_dist):
    """Offset a plane in the direction of the z-axis.

    Parameters:
        plane (tuple): The first plane as a tuple of tuples (origin, x-axis, y-axis, z-axis)
        offset_dist (float): The distance to offset the plane.

    Returns:
        offset_plane (tuple): The offset plane as a tuple of tuples (origin, x-axis, y-axis, z-axis)
    """
    origin = vec_add(plane[0], vec_force_length(plane[3], offset_dist))

    return (origin, plane[1], plane[2], plane[3])


def offset_plane(plane, distance):
    """Offset a plane in the direction of the z-axis (used as an alias to plane_offset).

    Parameters:
        plane (tuple): The first plane as a tuple of tuples (origin, x-axis, y-axis, z-axis)
        offset_dist (float): The distance to offset the plane.

    Returns:
        offset_plane (tuple): The offset plane as a tuple of tuples (origin, x-axis, y-axis, z-axis)
    """
    return plane_offset(plane, distance)

# ______________________________________________________________________________________________________________________
# 'Box' Operations using tuples

def box_offset_vertices(vertices, offset_distance=3.0):
    """Offset eight vertices with the topology of a box (it can be skewed).  Positive offsets are outwards, negative inwards.

    Parameters:
        vertices (iterable): Eight vertices, such that 0,1,2,3 cycles a face and 4,5,6,7 are 'above' the first four vertices
        offset_dist (float): The distance to offset the box. Positive for outwards offsets, negative for inwards.

    Returns:
        offset_vertices (tuple): The eight vertices in theor newly offset positions
    """

    offset_vertices = [None] * 8

    # Even offset
    offsets = (offset_distance, offset_distance, offset_distance)

    offset_vertices[0] = offset_corner_3d(vertices[0], vertices[1], vertices[4], vertices[3], offsets)
    offset_vertices[1] = offset_corner_3d(vertices[1], vertices[2], vertices[5], vertices[0], offsets)
    offset_vertices[2] = offset_corner_3d(vertices[2], vertices[3], vertices[6], vertices[1], offsets)
    offset_vertices[3] = offset_corner_3d(vertices[3], vertices[0], vertices[7], vertices[2], offsets)
    offset_vertices[4] = offset_corner_3d(vertices[4], vertices[0], vertices[5], vertices[7], offsets)
    offset_vertices[5] = offset_corner_3d(vertices[5], vertices[1], vertices[6], vertices[4], offsets)
    offset_vertices[6] = offset_corner_3d(vertices[6], vertices[2], vertices[7], vertices[5], offsets)
    offset_vertices[7] = offset_corner_3d(vertices[7], vertices[3], vertices[4], vertices[6], offsets)

    return offset_vertices

if (__name__ == "__main__"):
    pass
    # test = vecs_average(((0,3,0), (0,4,0),(0,2,0)))
    # test = line_line_intersection_2D( ((0,0,0),(1,1,0)), ((0,1,0),(1,0,0)) )
    # test = angle_between_vecs((1.0,0.0,0.0),(1.0,0.6,0.0))
    # test = vec_lerp((0,0,0),(0,2,0),0.2)
    # test = project_point_towards((0,0,0),(2,2,0),3)
    # test = law_cosines_angle(3, 4, 5)
    # test = vec_rotate( (125.68,72.82,0.00), (39.65,19.62,21.01), (73.48,19.62,61.78), -90)
    # test = vec_dot((1, 1, 0), (1, 0, 0))
    # test = vec_cross((1,0,0),(0,1,0))
    # test = plane_from_points((0,0,0), (1,0,0), (0,1,0))
    # closest, dist = closest_point_index((0,0,0), ((0,10,0),(0,2,0),(10,10,0),(20,0,0),(100,0,0)), True)
    # min = min_distance((0,0,0), ((0,10,0),(0,2,0),(10,10,0),(20,0,0),(100,0,0)))
    # print closest, dist
    # print min
    # test = vec_lerp((0,0,2),(0,0,1),0.5)
    #test = plane_plane_intersection(((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
    #                                ((0, 0, 0), (0, -1, 0), (0, 0, 1), (-1, 0, 0)))
    #test = chamfer_corner_length((10,0,10), (11,0,10), (11,1,10), 1.414)
    #test = vecs_average([(0,1,1)])
    #test = q_round_coord((20.000000018391083901,10,23444455.11111111111), .001)
    #test = line_closest_point_unbounded((0,0,0), (0,1,0), (1,30,0))

    #test = centre_3_point_circle((15.1, 8.8, 23.7), (12.7, 0.1, -9.5), (46.0, 18.0, 11.0))
    #test = project_vector_onto_plane((0,5,1), (0,1,0))
    #test = distance_weighted_value((0,0,0), ((1,0,0),(0,1,0),(0,0,1),(1,0,0)), (1,1,1,1))
    test = angle_with_negative_lines(((0,0,0),(1,0,0)), ((0,0,0),(0,1,0)), (0,0,1))
    print test
