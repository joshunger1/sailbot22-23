"""
This file will be used for all the high level waypoint algorithms
If you have any questions about the code please reach out to:
Joshua Unger
OR
Owen Pfannenstiehl
"""

"""
Imports
"""
import numpy as np
import matplotlib as plt
from turfpy.measurement import boolean_point_in_polygon
from geojson import Point, Polygon, Feature



"""
SubClasses
"""
###########
# Coordinate Class
###########
class Coordinate():

    def __init__(self, lat: int, lng: int):
        self.lat = lat
        self.lng = lng


"""
Main Algorithm
"""
def createPolygonArray(arr):
    newArr = []
    for coord in arr:
        newArr.append((coord.lat, coord.lng))
    return newArr
    


val = input("How many verticies will the bounding box contain? ")
boundingBoxCoordinates = []

for x in range(int(val)):
    newCoordLat = input("Latitude:")
    newCoordLng = input("Longitude:")
    waypointNum = x+1
    waypointFinal = Coordinate(newCoordLat, newCoordLng)
    boundingBoxCoordinates.append(waypointFinal)
    print("Created waypoint number " + str(waypointNum) + " with latitude of " + str(newCoordLat) + " and longitude of " + str(newCoordLng) + ".")

tupleCoordinateArr = []
for coord in boundingBoxCoordinates:
    coordtuple = tuple([float(coord.lat), float(coord.lng)])
    tupleCoordinateArr.append(coordtuple)

print(tupleCoordinateArr)
desiredLat = input("Latitude:")
desiredLng = input("Longitude:")
desiredWaypoint = Coordinate(newCoordLat, newCoordLng)

point = Feature(geometry=Point((float(desiredWaypoint.lat), float(desiredWaypoint.lng))))

polygon = Polygon(
    [
        tupleCoordinateArr
    ]
)
isWithin = boolean_point_in_polygon(point, polygon)
print(isWithin)
