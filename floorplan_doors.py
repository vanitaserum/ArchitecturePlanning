#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct  2 22:49:03 2025

@author: vanidutta
"""

# Sample data
floorplan_1 = {
    "id": "1",
    "name": "One Bedroom Apartment",
    "dimensions": {
        "width": 50,  # feet
        "height": 30,  # feet
    },
    "exteriorWalls": [
        {"start": {"x": 0, "y": 0}, "end": {"x": 50, "y": 0}, "thickness": 0.5},
        {"start": {"x": 50, "y": 0}, "end": {"x": 50, "y": 30}, "thickness": 0.5},
        {"start": {"x": 50, "y": 30}, "end": {"x": 0, "y": 30}, "thickness": 0.5},
        {"start": {"x": 0, "y": 30}, "end": {"x": 0, "y": 0}, "thickness": 0.5},
    ],
    "rooms": [
        {
            "id": "kitchen",
            "name": "Kitchen",
            "type": "kitchen",
            "walls": [
                {"start": {"x": 0, "y": 30}, "end": {"x": 20, "y": 30}, "thickness": 0.3},
                {"start": {"x": 20, "y": 30}, "end": {"x": 20, "y": 20}, "thickness": 0.3},
                {"start": {"x": 20, "y": 20}, "end": {"x": 0, "y": 20}, "thickness": 0.3},
                {"start": {"x": 0, "y": 20}, "end": {"x": 0, "y": 30}, "thickness": 0.3},
            ],
        },
        {
            "id": "living",
            "name": "Living",
            "type": "living",
            "walls": [
                {"start": {"x": 20, "y": 30}, "end": {"x": 40, "y": 30}, "thickness": 0.3},
                {"start": {"x": 40, "y": 30}, "end": {"x": 40, "y": 20}, "thickness": 0.3},
                {"start": {"x": 40, "y": 20}, "end": {"x": 20, "y": 20}, "thickness": 0.3},
                {"start": {"x": 20, "y": 20}, "end": {"x": 20, "y": 30}, "thickness": 0.3},
            ],
        },
        {
            "id": "bathroom_1",
            "name": "Bathroom 1",
            "type": "bathroom",
            "walls": [
                {"start": {"x": 20, "y": 20}, "end": {"x": 30, "y": 20}, "thickness": 0.3},
                {"start": {"x": 30, "y": 20}, "end": {"x": 30, "y": 10}, "thickness": 0.3},
                {"start": {"x": 30, "y": 10}, "end": {"x": 20, "y": 10}, "thickness": 0.3},
                {"start": {"x": 20, "y": 10}, "end": {"x": 20, "y": 20}, "thickness": 0.3},
            ],
        },
        {
            "id": "bedroom",
            "name": "Bedroom",
            "type": "bedroom",
            "walls": [
                {"start": {"x": 30, "y": 20}, "end": {"x": 50, "y": 20}, "thickness": 0.3},
                {"start": {"x": 50, "y": 20}, "end": {"x": 50, "y": 10}, "thickness": 0.3},
                {"start": {"x": 50, "y": 10}, "end": {"x": 30, "y": 10}, "thickness": 0.3},
                {"start": {"x": 30, "y": 10}, "end": {"x": 30, "y": 20}, "thickness": 0.3},
            ],
        },
        {
            "id": "bathroom_2",
            "name": "Bathroom 2",
            "type": "bathroom",
            "walls": [
                {"start": {"x": 40, "y": 30}, "end": {"x": 50, "y": 30}, "thickness": 0.3},
                {"start": {"x": 50, "y": 30}, "end": {"x": 50, "y": 20}, "thickness": 0.3},
                {"start": {"x": 50, "y": 20}, "end": {"x": 40, "y": 20}, "thickness": 0.3},
                {"start": {"x": 40, "y": 20}, "end": {"x": 40, "y": 30}, "thickness": 0.3},
            ],
        },
    ],
}
floorplan_3 = {
    "id": "3",
    "name": "Two Bedroom Apartment",
    "dimensions": {
        "width": 40,  # feet
        "height": 30,  # feet
    },
    "exteriorWalls": [
        {"start": {"x": 0, "y": 0}, "end": {"x": 40, "y": 0}, "thickness": 0.5},
        {"start": {"x": 40, "y": 0}, "end": {"x": 40, "y": 30}, "thickness": 0.5},
        {"start": {"x": 40, "y": 30}, "end": {"x": 0, "y": 30}, "thickness": 0.5},
        {"start": {"x": 0, "y": 30}, "end": {"x": 0, "y": 0}, "thickness": 0.5},
    ],
    "rooms": [
        {
            "id": "bedroom_1",
            "name": "Bedroom 1",
            "type": "bedroom",
            "walls": [
                {"start": {"x": 0, "y": 30}, "end": {"x": 20, "y": 30}, "thickness": 0.3},
                {"start": {"x": 20, "y": 30}, "end": {"x": 20, "y": 15}, "thickness": 0.3},
                {"start": {"x": 20, "y": 15}, "end": {"x": 10, "y": 15}, "thickness": 0.3},
                {"start": {"x": 10, "y": 15}, "end": {"x": 10, "y": 20}, "thickness": 0.3},
                {"start": {"x": 10, "y": 20}, "end": {"x": 0, "y": 20}, "thickness": 0.3},
                {"start": {"x": 0, "y": 20}, "end": {"x": 0, "y": 30}, "thickness": 0.3},
            ],
        },
        {
            "id": "bathroom_1",
            "name": "Bathroom 1",
            "type": "bathroom",
            "walls": [
                {"start": {"x": 0, "y": 20}, "end": {"x": 10, "y": 20}, "thickness": 0.3},
                {"start": {"x": 10, "y": 20}, "end": {"x": 10, "y": 10}, "thickness": 0.3},
                {"start": {"x": 10, "y": 10}, "end": {"x": 0, "y": 10}, "thickness": 0.3},
                {"start": {"x": 0, "y": 10}, "end": {"x": 0, "y": 20}, "thickness": 0.3},
            ],
        },
        {
            "id": "bedroom_2",
            "name": "Bedroom 2",
            "type": "bedroom",
            "walls": [
                {"start": {"x": 0, "y": 10}, "end": {"x": 10, "y": 10}, "thickness": 0.3},
                {"start": {"x": 10, "y": 10}, "end": {"x": 10, "y": 15}, "thickness": 0.3},
                {"start": {"x": 10, "y": 15}, "end": {"x": 20, "y": 15}, "thickness": 0.3},
                {"start": {"x": 20, "y": 15}, "end": {"x": 20, "y": 0}, "thickness": 0.3},
                {"start": {"x": 20, "y": 0}, "end": {"x": 0, "y": 0}, "thickness": 0.3},
                {"start": {"x": 0, "y": 0}, "end": {"x": 0, "y": 10}, "thickness": 0.3},
            ],
        },
        {
            "id": "kitchen",
            "name": "Kitchen",
            "type": "kitchen",
            "walls": [
                {"start": {"x": 20, "y": 30}, "end": {"x": 40, "y": 30}, "thickness": 0.3},
                {"start": {"x": 40, "y": 30}, "end": {"x": 40, "y": 20}, "thickness": 0.3},
                {"start": {"x": 40, "y": 20}, "end": {"x": 20, "y": 20}, "thickness": 0.3},
                {"start": {"x": 20, "y": 20}, "end": {"x": 20, "y": 30}, "thickness": 0.3},
            ],
        },
        {
            "id": "bathroom_2",
            "name": "Bathroom 2",
            "type": "bathroom",
            "walls": [
                {"start": {"x": 30, "y": 20}, "end": {"x": 40, "y": 20}, "thickness": 0.3},
                {"start": {"x": 40, "y": 20}, "end": {"x": 40, "y": 10}, "thickness": 0.3},
                {"start": {"x": 40, "y": 10}, "end": {"x": 30, "y": 10}, "thickness": 0.3},
                {"start": {"x": 30, "y": 10}, "end": {"x": 30, "y": 20}, "thickness": 0.3},
            ],
        },
        {
            "id": "living",
            "name": "Living",
            "type": "living",
            "walls": [
                {"start": {"x": 20, "y": 20}, "end": {"x": 30, "y": 20}, "thickness": 0.3},
                {"start": {"x": 30, "y": 20}, "end": {"x": 30, "y": 10}, "thickness": 0.3},
                {"start": {"x": 30, "y": 10}, "end": {"x": 40, "y": 10}, "thickness": 0.3},
                {"start": {"x": 40, "y": 10}, "end": {"x": 40, "y": 0}, "thickness": 0.3},
                {"start": {"x": 40, "y": 0}, "end": {"x": 20, "y": 0}, "thickness": 0.3},
                {"start": {"x": 20, "y": 0}, "end": {"x": 20, "y": 20}, "thickness": 0.3},
            ],
        },
    ],
}

floorplan_6 = {
    "id": "6",
    "name": "Two Bedroom Home with Courtyard",
    "dimensions": {
        "width": 50,  # feet
        "height": 30,  # feet
    },
    "exteriorWalls": [
        {"start": {"x": 0, "y": 0}, "end": {"x": 50, "y": 0}, "thickness": 0.5},
        {"start": {"x": 50, "y": 0}, "end": {"x": 50, "y": 30}, "thickness": 0.5},
        {"start": {"x": 50, "y": 30}, "end": {"x": 0, "y": 30}, "thickness": 0.5},
        {"start": {"x": 0, "y": 30}, "end": {"x": 0, "y": 0}, "thickness": 0.5},
    ],
    "rooms": [
        {
            "id": "bathroom_1",
            "name": "Bathroom 1",
            "type": "bathroom",
            "walls": [
                {"start": {"x": 0, "y": 30}, "end": {"x": 10, "y": 30}, "thickness": 0.3},
                {"start": {"x": 10, "y": 30}, "end": {"x": 10, "y": 20}, "thickness": 0.3},
                {"start": {"x": 10, "y": 20}, "end": {"x": 0, "y": 20}, "thickness": 0.3},
                {"start": {"x": 0, "y": 20}, "end": {"x": 0, "y": 30}, "thickness": 0.3},
            ],
        },
        {
            "id": "bathroom_2",
            "name": "Bathroom 2",
            "type": "bathroom",
            "walls": [
                {"start": {"x": 10, "y": 30}, "end": {"x": 20, "y": 30}, "thickness": 0.3},
                {"start": {"x": 20, "y": 30}, "end": {"x": 20, "y": 20}, "thickness": 0.3},
                {"start": {"x": 20, "y": 20}, "end": {"x": 10, "y": 20}, "thickness": 0.3},
                {"start": {"x": 10, "y": 20}, "end": {"x": 10, "y": 30}, "thickness": 0.3},
            ],
        },
        {
            "id": "bedroom_1",
            "name": "Bedroom 1",
            "type": "bedroom",
            "walls": [
                {"start": {"x": 0, "y": 20}, "end": {"x": 20, "y": 20}, "thickness": 0.3},
                {"start": {"x": 20, "y": 20}, "end": {"x": 20, "y": 0}, "thickness": 0.3},
                {"start": {"x": 20, "y": 0}, "end": {"x": 0, "y": 0}, "thickness": 0.3},
                {"start": {"x": 0, "y": 0}, "end": {"x": 0, "y": 20}, "thickness": 0.3},
            ],
        },
        {
            "id": "bedroom_2",
            "name": "Bedroom 2",
            "type": "bedroom",
            "walls": [
                {"start": {"x": 20, "y": 30}, "end": {"x": 40, "y": 30}, "thickness": 0.3},
                {"start": {"x": 40, "y": 30}, "end": {"x": 40, "y": 25}, "thickness": 0.3},
                {"start": {"x": 40, "y": 25}, "end": {"x": 30, "y": 25}, "thickness": 0.3},
                {"start": {"x": 30, "y": 25}, "end": {"x": 30, "y": 15}, "thickness": 0.3},
                {"start": {"x": 30, "y": 15}, "end": {"x": 20, "y": 15}, "thickness": 0.3},
                {"start": {"x": 20, "y": 15}, "end": {"x": 20, "y": 30}, "thickness": 0.3},
            ],
        },
        {
            "id": "living",
            "name": "Living",
            "type": "living",
            "walls": [
                {"start": {"x": 20, "y": 10}, "end": {"x": 40, "y": 10}, "thickness": 0.3},
                {"start": {"x": 40, "y": 10}, "end": {"x": 40, "y": 0}, "thickness": 0.3},
                {"start": {"x": 40, "y": 0}, "end": {"x": 20, "y": 0}, "thickness": 0.3},
                {"start": {"x": 20, "y": 0}, "end": {"x": 20, "y": 10}, "thickness": 0.3},
            ],
        },
        {
            "id": "kitchen",
            "name": "Kitchen",
            "type": "kitchen",
            "walls": [
                {"start": {"x": 40, "y": 30}, "end": {"x": 50, "y": 30}, "thickness": 0.3},
                {"start": {"x": 50, "y": 30}, "end": {"x": 50, "y": 5}, "thickness": 0.3},
                {"start": {"x": 50, "y": 5}, "end": {"x": 40, "y": 5}, "thickness": 0.3},
                {"start": {"x": 40, "y": 5}, "end": {"x": 40, "y": 30}, "thickness": 0.3},
            ],
        },
        {
            "id": "bathroom_3",
            "name": "Bathroom 3",
            "type": "bathroom",
            "walls": [
                {"start": {"x": 40, "y": 5}, "end": {"x": 50, "y": 5}, "thickness": 0.3},
                {"start": {"x": 50, "y": 5}, "end": {"x": 50, "y": 0}, "thickness": 0.3},
                {"start": {"x": 50, "y": 0}, "end": {"x": 40, "y": 0}, "thickness": 0.3},
                {"start": {"x": 40, "y": 0}, "end": {"x": 40, "y": 5}, "thickness": 0.3},
            ],
        },
    ],
}

import matplotlib.pyplot as plt
import numpy as np
import shapely.geometry as sg
from shapely import ops
import json

def order_walls(walls):
  '''
  if walls are not already ordered, this function orders walls so they form a continuous loop (polygon)
  each wall is a dictionary / object (json) with start and end coordinates
  returns an ordered list of [x,y] lists
  '''
  if not walls:
    return[]

  #starts with the first wall
  ordered = [walls[0]]
  walls_left = walls[1:]

#loops until all the walls have been ordered
  while walls_left:
    last_end = ordered[-1]['end']
    found = False

    #finds the next wall that connects to the current end point
    for i, wall in enumerate(walls_left):
      if last_end == wall['start']:
        ordered.append(walls_left.pop(i))
        found = True
        break

      #checks if the wall needs to be flipped
      elif last_end == wall['end']: #flips the wall if needed
        wall = {'start': wall['end'], 'end':wall['start']}
        ordered.append(wall)
        walls_left.pop(i)
        found = True
        break

  #converts the wall segments to a list of coordinates
  new_wall_coords = [[w['start']['x'], w['start']['y']] for w in ordered]
  new_wall_coords.append([ordered[-1]['end']['x'], ordered[-1]['end']['y']])
  return new_wall_coords

def find_shared_walls(rooms, tol=0.1):
  '''
  finds where rooms share walls
  tol is the condition for non-trivial wall intersections
  '''
  shared_walls = []

  #checks all UNIQUE pairs of rooms
  for i, room1 in enumerate(rooms):
        for j, room2 in enumerate(rooms):
            if i >= j:  # makes sure each room is only checked once
                continue

            #create polygons from boundaries of rooms
            poly1 = sg.Polygon(room1['polygon'])
            poly2 = sg.Polygon(room2['polygon'])

            #find where these rooms/polygons intersect
              #(this is where a door should go)
            intersection = poly1.intersection(poly2)

            valid_segments = []


            if intersection.geom_type == "LineString" and intersection.length > tol:
                valid_segments.append(intersection)
                #shared_walls.append(((room1['id'], room2['id']), intersection))

            #handles multiple wall segments that are disconnected
             #(i tried to do this with just linestring and it broke so i added more options)
            elif intersection.geom_type == "MultiLineString":
                for line in intersection.geoms:
                    if line.length > tol:
                      valid_segments.append(line)
                        #shared_walls.append(((room1['id'], room2['id']), line))

            elif intersection.geom_type == "GeometryCollection":
                # should handle collections that are more complicated
                for geom in intersection.geoms:
                    if geom.geom_type == "LineString" and geom.length > tol:
                        valid_segments.append(geom)
                        #shared_walls.append(((room1['id'], room2['id']), geom))

            #for the valid line segments checks where the wall should be placed if there are multiple connects between rooms
            if valid_segments:
                if len(valid_segments) == 1:
                    shared_walls.append(((room1['id'], room2['id']), valid_segments[0]))
                else:
                 longest_segment = max(valid_segments, key=lambda seg: seg.length)
                 shared_walls.append(((room1['id'], room2['id']), longest_segment))

  return shared_walls

def detect_openings(floorplan_data, rooms_data):
    """
    detect openings/empty space by finding walls that face empty space
    """
    courtyard_openings = []

    #creates polygons for all rooms
    room_polygons = []
    for room in rooms_data:
        if 'polygon' in room:
            room_polygons.append(sg.Polygon(room['polygon']))

    if not room_polygons:
        return courtyard_openings

    # create a union of all room polygons
    all_rooms_union = ops.unary_union(room_polygons)

    # create the building boundary
    building_bounds = [
        (0, 0),
        (floorplan_data["dimensions"]["width"], 0),
        (floorplan_data["dimensions"]["width"], floorplan_data["dimensions"]["height"]),
        (0, floorplan_data["dimensions"]["height"]),
        (0, 0)
    ]
    building_polygon = sg.Polygon(building_bounds)

    # the 'courtyard' is the space inside the building but outside all rooms
    #this could also be a random living space but im just calling them all courtyards
    courtyard_polygon = building_polygon.difference(all_rooms_union)

    if courtyard_polygon.is_empty:
        return courtyard_openings

    # finds which room walls face the courtyard
    for room in rooms_data:
        if 'polygon' not in room:
            continue

        room_poly = sg.Polygon(room['polygon'])

        #checks each wall segment to see if it faces the courtyard
        for i in range(len(room['polygon']) - 1):
            wall_start = room['polygon'][i]
            wall_end = room['polygon'][i + 1]
            wall_line = sg.LineString([wall_start, wall_end])

            #creates a small buffer outward from the wall to check what's on the other side
            wall_buffer = wall_line.buffer(0.5, cap_style=2)  # 0.5 foot buffer

            # checks if this buffer intersects with the courtyard
            if wall_buffer.intersects(courtyard_polygon):
                intersection = wall_buffer.intersection(courtyard_polygon)
                if intersection.area > 0.1:  # Significant intersection
                    courtyard_openings.append(((room['id'], 'courtyard'), wall_line))

    return courtyard_openings


def place_doors(shared_walls, rooms_data):
  '''
  places doors on shared walls based on room types and connections
  '''
  doors = []
  door_length = 2

  #creates diectionaries that hold room information
  room_types = {}
  room_names = {}
  for room in rooms_data:
      room_types[room['id']] = room['type']
      room_names[room['id']] = room['name']

  #defines which connects (eg; bedroom to bedroom) shouldn't have doors
  forbidden_connections = [
      ('bedroom', 'bedroom'),
      ('bathroom', 'bathroom'),
      # add courtyard restrictions
      ('bedroom', 'courtyard'),
      ('bathroom', 'courtyard'),
    ]

  #puts doors at midpoints of shared segments
  for (room1, room2), wall in shared_walls:
    midpoint = wall.interpolate(0.5, normalized = True) #0.5 because we want it halfway across the intersection

    #determines if the room gets a door or an archway
    room1_type = room_types.get(room1, '')
    room2_type = room_types.get(room2, '')
    room1_name = room_names.get(room1, '').lower()
    room2_name = room_names.get(room2, '').lower()

    #checks if connection is to 'courtyard'
    is_courtyard_connection = (room2 == 'courtyard' or 'courtyard' in room1_type or 'courtyard' in room2_type)

    if is_courtyard_connection:
        # if either room is a bedroom or bathroom and we're connecting to courtyard, skip
      if room1_type in ['bedroom', 'bathroom'] or room2_type in ['bedroom', 'bathroom']:
          continue
      else:
            # for regular room-to-room connections should just use the forbidden connections list

    #skip forbidden room connections
        connection = tuple(sorted([room1_type, room2_type]))
        if connection in forbidden_connections:
          continue


    #checks if bedroom and bathroom are reduntantly connected
    if ((room1_type == 'bedroom' and room2_type == 'bedroom') or (room1_type == 'bathroom' and room2_type == 'bathroom')):
      continue  # Skip to next wall, don't place a door

    #calculates the position and orientation of the doors
    coords = list(wall.coords)
    if len(coords) >= 2:
      start_x, start_y = coords[0]
      end_x, end_y = coords[-1]

      #determine if wall is horizontal or vertical
      is_horizontal = abs(start_y - end_y) < 0.1
      is_vertical = abs(start_x - end_x) < 0.1

      if is_horizontal: #horizontal door
        door_start = (midpoint.x - door_length/2, midpoint.y)
        door_end = (midpoint.x + door_length/2, midpoint.y)
        text_offset = (0, -1)  # Below the wall
        ha = 'center'
        va = 'top'
        rotation = 0

      elif is_vertical: #vertical door
        door_start = (midpoint.x, midpoint.y - door_length/2)
        door_end = (midpoint.x, midpoint.y + door_length/2)
        text_offset = (1, 0)   # To the right of the wall
        ha = 'left'
        va = 'center'
        rotation = 90 #so that it aligns with the wall

    else: #for anything else (idk diagonal or smth??)
      door_start = (midpoint.x - door_length/2, midpoint.y)
      door_end = (midpoint.x + door_length/2, midpoint.y)
      text_offset = (0, -1)
      ha = 'center'
      va = 'top'
      rotation = 0

    is_door = (room1_type in ['bedroom', 'bathroom'] or room2_type in ['bedroom', 'bathroom'])
    is_courtyard = ('courtyard' in room1_type or 'courtyard' in room2_type)

    if is_courtyard:
        door_label = "Archway"
        is_archway = True

    else:
        door_label = "Door" if is_door else "Archway"
        is_archway = not is_door


    door_label = "Door" if is_door else "Archway"

    doors.append({
        'room1': room1,
        'room2': room2,
        'position': (midpoint.x, midpoint.y),
        'door_start': door_start,
        'door_end': door_end,
        'text_position': (midpoint.x + text_offset[0], midpoint.y + text_offset[1]),
        'ha': ha,
        'va': va,
        'rotation': rotation,
        'label': door_label,
        'is_archway': is_archway
    })


  return doors

def generate_doors(floorplan_data):
  '''
  generates doors based on the given floorplan
  '''
#get the data!
  data = floorplan_data
  rooms = data['rooms']

#now we convert the walls to polygons!
  for room in rooms:
    if 'walls' in room:
      room['polygon'] = order_walls(room['walls'])

  courtyard_openings = detect_openings(data, rooms)
  shared_walls = find_shared_walls(rooms)

  all_shared_walls = shared_walls + courtyard_openings
  doors = place_doors(all_shared_walls, rooms)
  return rooms, doors


# Visualization

def calculate_polygon_centroid(vertices):
    n = len(vertices)
    if n < 3:
        return None  # Not a polygon

    area = 0.0
    cx = 0.0
    cy = 0.0

    for i in range(n):
        j = (i + 1) % n
        x_i, y_i = vertices[i]['x'], vertices[i]['y']
        x_j, y_j = vertices[j]['x'], vertices[j]['y']

        factor = x_i * y_j - x_j * y_i
        area += factor
        cx += (x_i + x_j) * factor
        cy += (y_i + y_j) * factor

    area /= 2.0
    if area == 0:
      return None # Avoid division by zero for degenerate polygons

    cx /= (6.0 * area)
    cy /= (6.0 * area)

    return cx, cy


#this was modified to plot doors too
def plot_floorplan(floorplan, doors=None):
    # Create a new figure and axis
    # Plot exterior walls
    for wall in floorplan["exteriorWalls"]:
        x_values = [wall["start"]["x"], wall["end"]["x"]]
        y_values = [wall["start"]["y"], wall["end"]["y"]]
        plt.plot(x_values, y_values, color='black', linewidth=wall["thickness"]*5) # Scale thickness for visibility

    # Plot interior walls (rooms)
    for room in floorplan["rooms"]:
        for wall in room["walls"]:
            x_values = [wall["start"]["x"], wall["end"]["x"]]
            y_values = [wall["start"]["y"], wall["end"]["y"]]
            plt.plot(x_values, y_values, color='gray', linewidth=wall["thickness"]*5) # Scale thickness for visibility

    # Set plot limits and aspect ratio
    plt.xlim(0, floorplan["dimensions"]["width"])
    plt.ylim(0, floorplan["dimensions"]["height"])
    plt.gca().set_aspect('equal', adjustable='box')

    # Add title and labels (optional)
    plt.title(f'Floorplan: {floorplan["name"]}')
    plt.xlabel('X (feet)')
    plt.ylabel('Y (feet)')

    # Add labels to the centroid of each room
    for room in floorplan["rooms"]:
                # Extract vertices for polygon centroid calculation
        vertices = []
        for wall in room["walls"]:
            vertices.append(wall["start"])
        # Add the last endpoint to close the polygon
        if room["walls"]:
             vertices.append(room["walls"][-1]["end"])

        # Calculate and plot centroid
        centroid = calculate_polygon_centroid(vertices)
        if centroid:
            plt.text(centroid[0], centroid[1], room["name"], ha='center', va='center', fontsize=8)

#adds the doors
    rooms_data, doors_data = generate_doors(floorplan)
    if doors_data:
      for door in doors_data:
        start_x, start_y = door['door_start']
        end_x, end_y = door['door_end']
        door_color = 'grey' if door.get('is_archway', False) else 'black'
        plt.plot([start_x, end_x], [start_y, end_y], color=door_color, linewidth=3, linestyle='-')

        room1_name = next((room['name'] for room in rooms_data if room['id'] == door['room1']), door['room1'])
        room2_name = next((room['name'] for room in rooms_data if room['id'] == door['room2']), door['room2'])
        door_label = door['label']

        text_x, text_y = door['text_position']
        plt.text(text_x, text_y, door_label, color='black', ha=door['ha'], va=door['va'], fontsize=6, rotation=door['rotation']),
        x, y = door['position']


    # Display the plot
    plt.show()
    

#PLOTTING
plot_floorplan(floorplan_1)
print("")
plot_floorplan(floorplan_3)
print("")
print("Note that floorplan 6 has a courtyard")
plot_floorplan(floorplan_6)
