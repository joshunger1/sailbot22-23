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

    #self explanitory, a class to hold both the latitude and longitude values for a location
    #potentially not required, can replace with a tuple
    def __init__(self, lat: int, lng: int):
        self.lat = lat
        self.lng = lng


"""
Main Algorithm
"""
#helper function to convert an array of coordinate objects
def createPolygonArray(arr):
    newArr = []
    for coord in arr:
        newArr.append((coord.lat, coord.lng))
    return newArr
    

#take user input for defining the bounary polygon that the robot will stay within 
val = input("How many verticies will the bounding box contain? ")
boundingBoxCoordinates = []

#for the amount of verticies that make up the polygon, take user input for the latitude and longitude for it
for x in range(int(val)):
    newCoordLat = input("Latitude:")
    newCoordLng = input("Longitude:")
    waypointNum = x+1
    waypointFinal = Coordinate(newCoordLat, newCoordLng)
    boundingBoxCoordinates.append(waypointFinal)
    print("Created waypoint number " + str(waypointNum) + " with latitude of " + str(newCoordLat) + " and longitude of " + str(newCoordLng) + ".")

#convert coordinates to a tuple so we can build the polygon
tupleCoordinateArr = []
for coord in boundingBoxCoordinates:
    coordtuple = tuple([float(coord.lat), float(coord.lng)])
    tupleCoordinateArr.append(coordtuple)

#collect information for the desired waypoint that the robot will sail to
print(tupleCoordinateArr)
desiredLat = input("Latitude:")
desiredLng = input("Longitude:")
desiredWaypoint = Coordinate(newCoordLat, newCoordLng)

#generate the polygon feature with the desired waypoint
point = Feature(geometry=Point((float(desiredWaypoint.lat), float(desiredWaypoint.lng))))

polygon = Polygon(
    [
        tupleCoordinateArr
    ]
)

#check if the desired waypoint is within the polygon
isWithin = boolean_point_in_polygon(point, polygon)
print(isWithin)
