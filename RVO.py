from math import ceil, floor, sqrt
import copy
import numpy

from math import cos, sin, tan, atan2, asin

from math import pi as PI

# Constants for phases
FORMATION_PHASE = 1
DESTINATION_PHASE = 2


def arrange_in_line(num_bots, start_point, spacing):
    """Generate a list of target positions for bots to arrange in a straight line."""
    formation_positions = []
    for i in range(num_bots):
        x_pos = start_point[0] + i * spacing
        y_pos = start_point[1]
        formation_positions.append([x_pos, y_pos])
    return formation_positions


def distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


def RVO_two_phase_update(X, V_des, V_current, ws_model, phase, formation_points=None):
    """ Two-phase movement: Formation first, then proceed to destination """
    if phase == FORMATION_PHASE:
        # Move to formation points
        V_new = []
        for i in range(len(X)):
            desired_formation = formation_points[i]
            V_new.append([desired_formation[0] - X[i][0], desired_formation[1] - X[i][1]])
        return RVO_update(X, V_new, V_current, ws_model)

    elif phase == DESTINATION_PHASE:
        # Move to final destination (V_des)
        return RVO_update(X, V_des, V_current, ws_model)


def check_formation_complete(X, formation_points, tolerance=0.1):
    """Check if all bots have reached their formation points."""
    for i in range(len(X)):
        if distance(X[i], formation_points[i]) > tolerance:
            return False
    return True




def distance(pose1, pose2):
    """ compute Euclidean distance for 2D """
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)+0.001
def distance_r(pose1, pose2, width1, height1, width2, height2):
    """
    Compute the distance between two rectangles for collision detection.
    This is an approximation that considers their bounding boxes.
    """
    dist_x = abs(pose1[0] - pose2[0]) - (width1 + width2) / 2
    dist_y = abs(pose1[1] - pose2[1]) - (height1 + height2) / 2
    if dist_x < 0 and dist_y < 0:
        return 0  # Overlapping, return 0 distance
    return max(dist_x, 0) + max(dist_y, 0) + 0.001


def RVO_update(X, V_des, V_current, ws_model):
    """ Compute the best velocity given desired velocity, current velocity, and workspace model """
    SAFETY_MARGIN = 1  # More conservative safety margin to ensure no collisions
    ROB_RAD = ws_model['robot_radius'] + SAFETY_MARGIN
    V_opt = list(V_current)

    for i in range(len(X)):
        vA = [V_current[i][0], V_current[i][1]]
        pA = [X[i][0], X[i][1]]
        width_A, height_A = ws_model['robot_dimensions'][i]
        RVO_BA_all = []

        for j in range(len(X)):
            if i != j:
                vB = [V_current[j][0], V_current[j][1]]
                pB = [X[j][0], X[j][1]]
                width_B, height_B = ws_model['robot_dimensions'][j]

                # Translating velocity
                transl_vB_vA = [pA[0] + 0.5 * (vB[0] + vA[0]), pA[1] + 0.5 * (vB[1] + vA[1])]

                # Calculate the safe separation distance with a margin
                dist_BA = distance_r(pA, pB, width_A, height_A, width_B, height_B)
                theta_BA = atan2(pB[1] - pA[1], pB[0] - pA[0])

                # Enforce a larger minimum separation (avoid last-second adjustments)
                MIN_SEPARATION = 4 * ROB_RAD  # Minimum distance they must maintain
                if dist_BA < MIN_SEPARATION:
                    dist_BA = MIN_SEPARATION  # Adjust to ensure safe separation

                theta_BAort = asin(MIN_SEPARATION / dist_BA)
                theta_ort_left = theta_BA + theta_BAort
                bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                theta_ort_right = theta_BA - theta_BAort
                bound_right = [cos(theta_ort_right), sin(theta_ort_right)]

                RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, MIN_SEPARATION]
                RVO_BA_all.append(RVO_BA)

        # For circular obstacles
        for hole in ws_model['circular_obstacles']:
            vB = [0, 0]
            pB = hole[0:2]
            transl_vB_vA = [pA[0] + vB[0], pA[1] + vB[1]]

            # Circular obstacle distance unchanged
            dist_BA = distance_r(pA, pB, width_A, height_A, hole[2], hole[2])
            theta_BA = atan2(pB[1] - pA[1], pB[0] - pA[0])

            # Over-approximation of square to circular obstacle
            OVER_APPROX_C2S = 1.5
            rad = hole[2] * OVER_APPROX_C2S
            if (rad + ROB_RAD) > dist_BA:
                dist_BA = rad + ROB_RAD

            theta_BAort = asin((rad + ROB_RAD) / dist_BA)
            theta_ort_left = theta_BA + theta_BAort
            bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
            theta_ort_right = theta_BA - theta_BAort
            bound_right = [cos(theta_ort_right), sin(theta_ort_right)]

            RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad + ROB_RAD]
            RVO_BA_all.append(RVO_BA)

        # Calculate velocity based on updated RVOs
        vA_post = intersect(pA, V_des[i], RVO_BA_all)

        # Adjust speed if too close
        if distance(X[i], pA) < MIN_SEPARATION:
            speed_reduction_factor = 0.5  # Reduce speed when too close
            V_opt[i] = [speed_reduction_factor * vA_post[0],
                        speed_reduction_factor * vA_post[1]]
        else:
            V_opt[i] = [0.9 * vA_post[0] + 0.1 * V_current[i][0],  # Smoothed velocity update
                        0.9 * vA_post[1] + 0.1 * V_current[i][1]]

    return V_opt


def intersect(pA, vA, RVO_BA_all):
    norm_v = distance(vA, [0, 0])
    suitable_V = []
    unsuitable_V = []

    # Sweep through possible velocities
    for theta in numpy.arange(0, 2 * PI, 0.05):
        for rad in numpy.arange(0.02, norm_v + 0.02, norm_v / 10.0):
            new_v = [rad * cos(theta), rad * sin(theta)]
            suit = True
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break
            if suit:
                suitable_V.append(new_v)
            else:
                unsuitable_V.append(new_v)

    if suitable_V:
        vA_post = min(suitable_V, key=lambda v: distance(v, vA))
    else:
        vA_post = min(unsuitable_V, key=lambda v: distance(v, vA))
    return vA_post


def compute_V_des(X, goal, V_max):
    V_des = []
    for i in range(len(X)):
        dif_x = [goal[i][k] - X[i][k] for k in range(2)]
        norm = distance(dif_x, [0, 0])
        norm_dif_x = [dif_x[k] * V_max[k] / norm for k in range(2)]
        V_des.append(norm_dif_x[:])
        if reach(X[i], goal[i], 0.1):
            V_des[i][0] = 0
            V_des[i][1] = 0
    return V_des

def reach(p1, p2, bound=0.5):
    return distance(p1, p2) < bound

def distance(pose1, pose2):
    return sqrt((pose1[0] - pose2[0])**2 + (pose1[1] - pose2[1])**2) + 0.001

def in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= PI:
        return theta_right <= theta_dif <= theta_left
    else:
        if theta_left < 0 and theta_right > 0:
            theta_left += 2 * PI
            if theta_dif < 0:
                theta_dif += 2 * PI
            return theta_right <= theta_dif <= theta_left
        elif theta_left > 0 and theta_right < 0:
            theta_right += 2 * PI
            if theta_dif < 0:
                theta_dif += 2 * PI
            return theta_left <= theta_dif <= theta_right
        return False


