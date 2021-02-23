#!/usr/bin/env python

import sys
import json
import rospy
import numpy as np

import matplotlib.pyplot as plt

from shapely.geometry import Point, LineString, MultiPoint, LinearRing
from shapely.geometry.polygon import Polygon

from std_msgs.msg import String, Float32MultiArray, Empty

visited_points={}

def make_path(start, end):
  path = LineString([start, end])
  return path

def nearest_polygon(start,end,polys):
  """
  Finds the index of the nearest polygon to the start points
  inputs: start point (start), polygon array (polys)
  output: index of nearest polygon in the array
  """
  nearest_dist = float('inf')
  nearest_idx = 0
  path = make_path(start,end)

  for idx, poly in enumerate(polys):
    if path.crosses(poly) and start.distance(poly)<nearest_dist:
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
    line = LineString([start, Point(coord[0],coord[1])])

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
  return [coords[idx-1],coords[(idx+1)%len(coords)]]

def outermost_points(polys, start, end):
  """
  Finds the outermost points of the nearest polygon that falls in line of sight of path
  inputs: polygon array (polys), path
  output: two outermost points on nearest polygon
  """

  nearest_idx = nearest_polygon(start,end,polys)
  
  #find visible vertices of the nearest polygon
  vis_points = visible_points(start, polys[nearest_idx])
  
  if len(vis_points)<2:
    return adjacent_points(polys[nearest_idx],(start.x,start.y))
    
  #find vertices on each side of the line
  sides=[[],[]]
  for point in vis_points:
    if LinearRing([start.coords[0], end.coords[0], point]).is_ccw:
      sides[0].append(point)
    else:
      sides[1].append(point)
  
  #find extreme points on each side
  outermost_points=[]
  path = make_path(start,end)
  for side in sides:
    farthest_dist_side = 0
    farthest_idx_side = 0
    if len(side)!=0:
      for idx, point in enumerate(side):
        if Point(point).distance(path)>farthest_dist_side:
          farthest_dist_side = Point(point).distance(path)
          farthest_idx_side = idx
      outermost_points.append(side[farthest_idx_side])
    else:
      outermost_points.append((None, None)) #i.e. whole side is invisible

  #find extreme points on each side
  return outermost_points

def find_paths(start, end, polys):
  """
  inputs: start point, end point, map - polygons
  output: a list of paths
  """
  tempo_polys = sorted(polys,key=lambda x: x.distance(start), reverse=False)[:] # sorted list of polygons by distance from start point
  temp_start = start

  planned_path = []

  while True:
    temp_polys = tempo_polys # discard previous polygon
    temp_path = make_path(temp_start,end)

    #checks if any polygon crosses path, if not, loop can break as there is a direct path to the end
    crosses = False
    for poly in temp_polys:
      crosses = True if temp_path.crosses(poly) == True else crosses

    if crosses:
      outermost_pts = outermost_points(temp_polys,Point(temp_start),Point(end))

      # to prevent oscillation between same locally optimal points, keep track of visited points
      if str(Point(outermost_pts[0])) in visited_points: 
        temp_end = Point(outermost_pts[1])
      elif str(Point(outermost_pts[1])) in visited_points:
        temp_end = Point(outermost_pts[0])
      else:
        flag = end.distance(Point(outermost_pts[0])) < end.distance(Point(outermost_pts[1]))
        temp_end = Point(outermost_pts[0]) if flag else Point(outermost_pts[1])

      new_paths = find_paths(temp_start,temp_end,temp_polys)
      for path in new_paths:
        planned_path.append(path)
      
      visited_points[str(temp_start)] = True
      temp_start = temp_end
    else:
      new_path = make_path(temp_start,end)
      planned_path.append(new_path)

      return planned_path

def listener(Polygons):
  rospy.init_node('main', anonymous=True)

  rate = rospy.Rate(100)
  start_sub=rospy.wait_for_message('/start_point', Float32MultiArray)
  start_point = (start_sub.data[0],start_sub.data[1]) #no z consideration

  print('start:',start_point)

  #for initial tests, we supply end point manually thru rostopic pub
  end_sub=rospy.wait_for_message('/end_point', Float32MultiArray)
  end_point = (end_sub.data[0],end_sub.data[1])

  print('end:',end_point)
  planned_path = find_paths(Point(start_point), Point(end_point), Polygons)

  for poly in Polygons:
    plt.plot(poly.exterior.xy[0],poly.exterior.xy[1], 'g')
  for path in planned_path:
    plt.plot(path.xy[0],path.xy[1], 'rx-')
  plt.show()

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
          featCoor = [[i[0]-data["offset"][0],i[1]-data["offset"][1]]
  for i in feature["geometry"]["coordinates"][0]]
      p = MultiPoint(featCoor[:-1])
      Polygons.append(Polygon(featCoor[:-1]))
  listener(Polygons)
