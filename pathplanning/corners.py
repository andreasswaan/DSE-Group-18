import numpy as np
from shapely.geometry import Point, LineString, Polygon
import shapely
import networkx as nx
import matplotlib.pyplot as plt
import math
import pickle
import os
def round_corner_arc(A, B, C, r, num_arc_points=20):
    """Returns points for a rounded arc at corner B with radius r, tangent to both lines.
    The arc always starts at P (on AB) and ends at Q (on BC).
    The center is computed from P, at 90 degrees to v1 (left/right depending on turn), at distance r."""

    v1 = np.array([A[0] - B[0], A[1] - B[1]])
    v2 = np.array([C[0] - B[0], C[1] - B[1]])
    v1 = v1 / np.linalg.norm(v1)     
    v2 = v2 / np.linalg.norm(v2)
    # External angle (angle between headings)
    theta = np.pi - np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
    d = r * np.tan(theta / 2)
    # Points on AB and BC at distance d from B
    P = (B[0] + v1[0]*d, B[1] + v1[1]*d)
    Q = (B[0] + v2[0]*d, B[1] + v2[1]*d)
    # Compute the direction for the center: rotate v1 by +/-90 deg
    cross = v1[0]*v2[1] - v1[1]*v2[0]
    if cross < 0:
        # Right turn: rotate v1 by -90 deg
        normal = np.array([v1[1], -v1[0]])
    else:
        # Left turn: rotate v1 by +90 deg
        normal = np.array([-v1[1], v1[0]])
    center = (P[0] + normal[0]*r, P[1] + normal[1]*r)
    # Start and end angles for the arc
    angle1 = np.arctan2(P[1] - center[1], P[0] - center[0])
    angle2 = np.arctan2(Q[1] - center[1], Q[0] - center[0])
    # Choose direction (shortest arc)
    angle_diff = (angle2 - angle1 + np.pi*3) % (2*np.pi) - np.pi
    if angle_diff > 0:
        arc_angles = np.linspace(angle1, angle2, num=num_arc_points)
    else:
        arc_angles = np.linspace(angle1, angle2, num=num_arc_points)[::-1]
    arc_x = center[0] + r * np.cos(arc_angles)
    arc_y = center[1] + r * np.sin(arc_angles)
    return P, Q, arc_x, arc_y

def simplify_path(waypoints, min_angle_deg=170):
    if len(waypoints) < 3:
        return waypoints
    new_path = [waypoints[0]]
    for i in range(1, len(waypoints) - 1):
        A, B, C = np.array(waypoints[i-1]), np.array(waypoints[i]), np.array(waypoints[i+1])
        v1 = A - B
        v2 = C - B
        v1 = v1 / np.linalg.norm(v1)
        v2 = v2 / np.linalg.norm(v2)
        angle_deg = np.degrees(np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0)))
        if angle_deg < min_angle_deg:
            new_path.append(tuple(B))
    new_path.append(waypoints[-1])
    return new_path

def downsample_path(path, every=4):
    return path[::every] + [path[-1]]

def rounded_corners_path(waypoints, r=0.1, num_arc_points=20, min_angle_deg=160):
    """
    Returns:
        - path_points: list of (x, y) tuples for the full rounded-corner path
        - P_list: list of P points (arc starts)
        - Q_list: list of Q points (arc ends)
    """
    if len(waypoints) < 2:
        return [], [], []

    path_points = []
    P_list = []
    Q_list = []

    prev_point = waypoints[0]
    path_points.append(prev_point)

    for i in range(1, len(waypoints) - 1):
        A, B, C = np.array(waypoints[i - 1]), np.array(waypoints[i]), np.array(waypoints[i + 1])
        v1 = A - B
        v2 = C - B
        v1 = v1 / np.linalg.norm(v1)
        v2 = v2 / np.linalg.norm(v2)
        angle_rad = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)

        if angle_deg > min_angle_deg:
            # Too straight: skip rounding
            path_points.append(tuple(B))
            prev_point = tuple(B)
            continue

        try:
            P, Q, arc_x, arc_y = round_corner_arc(A, B, C, r, num_arc_points)
        except Exception:
            path_points.append(tuple(B))
            prev_point = tuple(B)
            continue

        if np.hypot(prev_point[0] - P[0], prev_point[1] - P[1]) > 1e-8:
            path_points.append(P)

        arc_points = list(zip(arc_x, arc_y))
        if np.hypot(path_points[-1][0] - arc_points[0][0], path_points[-1][1] - arc_points[0][1]) < 1e-8:
            path_points.extend(arc_points[1:])
        else:
            path_points.extend(arc_points)

        P_list.append(P)
        Q_list.append(Q)
        prev_point = Q

    # Final straight segment to goal
    if len(waypoints) > 2:
        last = waypoints[-1]
        if np.hypot(prev_point[0] - last[0], prev_point[1] - last[1]) > 1e-8:
            path_points.append(last)
    else:
        path_points.append(waypoints[-1])

    return path_points, P_list, Q_list
