#!/usr/bin/env python

import sys
import json
import rospy
import numpy as np

import matplotlib.pyplot as plt

from shapely.geometry import Point, LineString, MultiPoint, LinearRing
from shapely.geometry.polygon import Polygon

from std_msgs.msg import String, Float32MultiArray, Empty

pub = rospy.Publisher('/planned_path', Float32MultiArray, queue_size=100)

def segment_optimize(polys, wp):
    op = [wp[0], wp[1]]
    if len(wp) >= 3:
        for i in range(2, len(wp)):
            line = LineString([op[len(op)-2], wp[i]])
            intersects = False
            for p in polys:
                if line.intersects(p):
                    shape = line.intersection(p)
                    if (shape.geom_type != 'Point' and shape.geom_type != 'MultiPoint'):
                        intersects = True
                        break

            if not intersects:
                op[len(op)-1] = wp[i]
            else:
                op.append(wp[i])

    return op

def make_path(start, end):
    path = LineString([start, end])
    return path


def nearest_polygon(start, end, polys):
    """
    Finds the index of the nearest polygon to the start points
    inputs: start point (start), polygon array (polys)
    output: index of nearest polygon in the array
    """
    nearest_dist = float('inf')
    nearest_idx = 0
    path = make_path(start, end)

    for idx, poly in enumerate(polys):
        if path.crosses(poly) and start.distance(poly) < nearest_dist:
            nearest_dist = start.distance(poly)
            nearest_idx = idx
    return nearest_idx


def visible_points(start, poly):
    """
    Finds the vertices of a polygon that are visible from the start point
    inputs: start point (start), polygon
    """
    vis_points = []
    for coord in list(poly.exterior.coords[:-1]):
        line = LineString([start, Point(coord[0], coord[1])])

        # vertex is visible if the line it forms with start point intersects the polygon at a point,
        # not a line or any other shape
        if line.intersection(poly).geom_type == 'Point':
            vis_points.append(coord)
    return vis_points


def adjacent_points(poly, vertex):
    """
    input: polygon, a vertex of the polygon
    output: adjacent points of a vertex in a polygon
    """
    coords = list(poly.exterior.coords)[:-1]
    idx = coords.index(vertex)
    return [coords[idx-1], coords[(idx+1) % len(coords)]]


def outermost_points(polys, start, end):
    """
    Finds the outermost points of the nearest polygon that falls in line of sight of path
    inputs: polygon array (polys), path
    output: two outermost points on nearest polygon
    """

    nearest_idx = nearest_polygon(start, end, polys)

    # find visible vertices of the nearest polygon
    vis_points = visible_points(start, polys[nearest_idx])

    if len(vis_points) < 2:
        return adjacent_points(polys[nearest_idx], (start.x, start.y))

    # find vertices on each side of the line
    sides = [[], []]
    for point in vis_points:
        if LinearRing([start.coords[0], end.coords[0], point]).is_ccw:
            sides[0].append(point)
        else:
            sides[1].append(point)

    # find extreme points on each side
    outermost_points = []
    path = make_path(start, end)
    for side in sides:
        farthest_dist_side = 0
        farthest_idx_side = 0
        if len(side) != 0:
            for idx, point in enumerate(side):
                if Point(point).distance(path) > farthest_dist_side:
                    farthest_dist_side = Point(point).distance(path)
                    farthest_idx_side = idx
            outermost_points.append(side[farthest_idx_side])
        else:
            # i.e. whole side is invisible
            outermost_points.append((None, None))

    # find extreme points on each side
    return outermost_points


def vp3_path(start, end, polys):
    visited_points = {}

    def find_paths(start, end, polys):
        """
        inputs: start point, end point, map - polygons
        output: a list of paths
        """
        tempo_polys = sorted(polys,
                             key=lambda x: x.distance(start),
                             reverse=False)[:]  # sorted list of polygons by distance from start point
        temp_start = start

        planned_path = []

        while True:
            temp_polys = tempo_polys  # discard previous polygon
            temp_path = make_path(temp_start, end)

            # checks if any polygon crosses path, if not, loop can break as there is a direct path to the end
            crosses = False
            for poly in temp_polys:
                crosses = True if temp_path.crosses(poly) == True else crosses

            if crosses:
                outermost_pts = outermost_points(
                    temp_polys, Point(temp_start), Point(end)
                )

                # to prevent oscillation between same locally optimal points, keep track of visited points
                if str(Point(outermost_pts[0])) in visited_points:
                    temp_end = Point(outermost_pts[1])
                elif str(Point(outermost_pts[1])) in visited_points:
                    temp_end = Point(outermost_pts[0])
                else:
                    flag = end.distance(Point(outermost_pts[0])) < end.distance(
                        Point(outermost_pts[1]))
                    temp_end = Point(outermost_pts[0]) if flag else Point(
                        outermost_pts[1])

                new_paths = find_paths(temp_start, temp_end, temp_polys)
                for path in new_paths:
                    planned_path.append(path)

                visited_points[str(temp_start)] = True
                temp_start = temp_end
            else:
                new_path = make_path(temp_start, end)
                planned_path.append(new_path)

                return planned_path

    planned_path = find_paths(start, end, polys)

    waypoints = []
    for i, p in enumerate(planned_path):
        points = list(p.coords)
        if i == 0:
            waypoints.append(points[0])
        waypoints.append(points[1])

    optimized_csvga_path = segment_optimize(
        polys, segment_optimize(polys, waypoints))

    return optimized_csvga_path

def listener(Polygons):
  rospy.init_node('waypoint_follower', anonymous=True)

  rate = rospy.Rate(100)
  start_sub=rospy.wait_for_message('/GPSdata', Float32MultiArray)
  start = (start_sub.data[0],start_sub.data[1]) #no z consideration

  # a good spread of points to use for demo
  waypoints = [(100,0),(66,70),(-9.7,-99.6)]

  fig, axs = plt.subplots(2)

  # for debug purposes
  pltw = [start]+waypoints

  for poly in Polygons:
    axs[0].plot(poly.exterior.xy[0],poly.exterior.xy[1], 'g')
  axs[0].plot([coord[0] for coord in pltw],[coord[1] for coord in pltw], 'rx-')

  info = Float32MultiArray()

  planned_path_unflattened = []
  for waypoint in waypoints:
    fl = False
    for poly in Polygons:
      if Point(waypoint).within(poly):
        fl = True
    if fl:
      continue
    planned_path_seg = vp3_path(Point(start), Point(waypoint), Polygons)
    planned_path_unflattened.append(planned_path_seg)
    start = waypoint
  
  planned_path_coords = [item for sublist in planned_path_unflattened for item in sublist]

  #for debug purposes
  for poly in Polygons:
    axs[1].plot(poly.exterior.xy[0],poly.exterior.xy[1], 'g')
  plt.plot([coord[0] for coord in planned_path_coords],[coord[1] for coord in planned_path_coords], 'rx-')
  plt.show()

  msg = [[coord[0],coord[1],start_sub.data[2],0,0,0,1.0,0,15,0.5,0.5] for coord in planned_path_coords]
  msg.append([planned_path_coords[-1][0],planned_path_coords[-1][1],start_sub.data[2],0,0,0,1.0,0,15,0.5,0.5])
        
  info.data = list(np.array(msg).reshape((11*len(msg),)))
  pub.publish(info)
  
  while not rospy.is_shutdown():
    rate.sleep()

if __name__ == '__main__':
  print("Node running")

  with open(sys.argv[1]) as f:
    data = json.load(f)

  Polygons = []

  for feature in data["features"]:
    if feature["geometry"]["type"] == "Polygon":
      # subtracting offsets here, the offsets are in the geojson file
      featCoor = [[(i[0]-data["offset"][0])*data["scale"], (i[1]-data["offset"][1])*data["scale"]]

  for i in feature["geometry"]["coordinates"][0]]
    p = MultiPoint(featCoor[:-1])
    Polygons.append(Polygon(featCoor[:-1]))
  
  #for debug purposes
  #for poly in Polygons:
  #  plt.plot(poly.exterior.xy[0],poly.exterior.xy[1], 'g')
  #plt.show()

  listener(Polygons)
