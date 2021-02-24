#!/usr/bin/env python

# Triangle Width Characterization with Adjacency

from collections import deque
import sys
import json
import numpy as np
import random
import rospy
from shapely.geometry import Point, Polygon, LineString, box
from shapely.ops import triangulate
from std_msgs.msg import String, Float32MultiArray, Empty

pub = rospy.Publisher('/geofence_status', String, queue_size=100)
polys = []
bboxes = []
poly_triangles = []
poly_adjacency_graphs = []
current_poly_triangle = [None, None]

# uses shapely polygons
def delta_bounding_box(poly, delta=0.2):
    x_min, y_min, x_max, y_max = poly.bounds
    delta_x = delta*(x_max-x_min)
    delta_y = delta*(y_max-y_min)
  
    return box(x_min-delta_x, y_min-delta_y, x_max+delta_x, y_max+delta_y)

def bbox_occupied(point, bbox):
    x_min, y_min, x_max, y_max = bbox.bounds
    x, y = point.x, point.y
    if x_min <= x <= x_max and y_min <= y <= y_max:
        return True
    else:
        return False
    
    
def triangle_occupied(point, triangle):
      x = point.x
      y = point.y
      triangle_points = list(triangle.exterior.coords)[:3]
      w1 = ((triangle_points[2][0]-triangle_points[1][0])*(y-triangle_points[1][1])-(triangle_points[2][1]-triangle_points[1][1])*(x-triangle_points[1][0])) / ((triangle_points[2][0]-triangle_points[1][0])*(triangle_points[0][1]-triangle_points[1][1])-(triangle_points[2][1]-triangle_points[1][1])*(triangle_points[0][0]-triangle_points[1][0]))
      w2 = ((triangle_points[0][0]-triangle_points[2][0])*(y-triangle_points[2][1])-(triangle_points[0][1]-triangle_points[2][1])*(x-triangle_points[2][0])) / ((triangle_points[0][0]-triangle_points[2][0])*(triangle_points[1][1]-triangle_points[2][1])-(triangle_points[0][1]-triangle_points[2][1])*(triangle_points[1][0]-triangle_points[2][0]))
      w3 = ((triangle_points[1][0]-triangle_points[0][0])*(y-triangle_points[0][1])-(triangle_points[1][1]-triangle_points[0][1])*(x-triangle_points[0][0])) / ((triangle_points[1][0]-triangle_points[0][0])*(triangle_points[2][1]-triangle_points[0][1])-(triangle_points[1][1]-triangle_points[0][1])*(triangle_points[2][0]-triangle_points[0][0]))
      
      if w1>0 and w2>0 and w3>0:
        return True
      else:
        return False    
    
# run only when bbox check verified
def BFS_PIP(point, root, graph, triangles): 
    visited = set()
    queue = deque([root])
    
    visited.add(root)
    
    while queue:
        current = queue.popleft()
        if triangle_occupied(point, triangles[current][0]):
            # point occupies a given triangle
            return triangles[current][1], current
            
        for adj_node in graph[current]:
            if adj_node not in visited:
                visited.add(adj_node)
                queue.append(adj_node)
    
    return None
    
    
def geofences_init():  
    # initialization step
    
    for poly in polys:
        # calculate bounding boxes for polygons, keep bboxes for occupancy check
        bbox = delta_bounding_box(poly)
        bboxes.append(bbox)
        
        # join bounding box and polygon 
        poly_points = np.array(poly.exterior.coords)
        
        min_x_points = poly_points[np.where(poly_points[:, 0]==min(poly_points[:, 0])]
        min_point = min_x_points[np.argmin(poly_points[:, 1])]
        
        shared_line = LineString([Point(bbox.bounds[:2], min_point])

        merged = poly.boundary.union(shared_line.union(bbox.boundary))
        
        # triangulating merged polygon
        merged_triangles = [(triangle, triangle.within(poly1)) for triangle in triangulate(merged)] # tuple of triangular polygon and boolean of whether within or without
        
        # generating adjacency list
        graph = {}
        
        for i, triangle in enumerate(merged_triangles):
            adjacent = set()
            for  j, other_triangle in enumerate(merged_triangles):
                if triangle[0].touches(other_triangle[0]):
                    adjacent.add(j)
                    
            graph[i] = adjacent
        
        poly_adjacency_graphs.append(graph)
        
        
def callback(data):
    print("Data received, processing")
    gps = data.data

    p = Point(gps[0], gps[1])
        
    for i, bbox in enumerate(bboxes):
        if bbox_occupied(p, bbox):
            root = current_poly_triangle[1] if current_poly_triangle[1] and current_poly_triangle[0]==i else random.randint(0, len(poly_triangles[i]))
            occupancy, triangle_number = BFS_PIP(p, root, poly_adjacency_graphs[i], poly_triangles[i])
            if occupancy:
                current_poly_triangle = [i, triangle_number]
                pub.publish("breached ", i)
                return
            else:
                continue
        

    if gps[2] < zfence[0] or gps[2] > zfence[1]:
        pub.publish("breached ", i)
        return

    print("not breached")
    current_poly_triangle = [None, None]
    pub.publish(" ")
    pass
    
    
def listener():
    rospy.init_node('breach_detector', anonymous=True)

    rate = rospy.Rate(100)
    rospy.Subscriber('/GPSdata', Float32MultiArray, callback)

    while not rospy.is_shutdown():
        rate.sleep()
        
        
if __name__ == '__main__':
    print("Node running")
    with open(sys.argv[1]) as f:
        data = json.load(f)

    for feature in data["features"]:
        if feature["geometry"]["type"] == "Polygon":
            #subtracting offsets here, the offsets are in the geojson file
            featCoor = [[i[0]-data["offset"][0],i[1]-data["offset"][1]] for i in feature["geometry"]["coordinates"][0]]
            print(featCoor)
            polys.append(Polygon(featCoor))
    
    listener()