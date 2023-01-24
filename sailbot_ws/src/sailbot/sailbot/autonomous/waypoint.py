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
val = input("Enter your value: ")
print(val)
