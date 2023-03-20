# -*- coding: utf-8 -*-
"""
Created on Wed Feb  1 13:32:49 2023

@author: Douglas Moore

Sailbot nav alg visualizer and simulator



degrees have no welcome home here, and radians reign supreme across the land


Coordinate system is cartesian, +y is north, +x is east
all units are SI base units. m, m/s, etc...
The origin is at some constant location on the water. Location of the origin should not effect the direction that the boat decides to sail in

it is assumed in this program that the boat can sail at a constant velocity of 1 m/s regardless of heading (except in the no-go zone)

physicsUpdate() progresses the universe forward by a length dt, which can be specified but is by default 1s
calcHeading() calculates the optimal heading for the boat to go in



"""

import numpy as np
from matplotlib import pyplot as plt


#input user desired restrictions 

minSailingAngle = np.pi / 3 #minimum sailing angle for the boat
Dpmin = -np.cos(minSailingAngle) #minimum dot product for the boat's sailing angle
maxCte = 20 #max crosstrack error, in meters, that we will allow for the boat when going directly upwind (from centerline! if you want the boat to wander from -10 to +10m off course, put in 10!)


#global variables, position and velocity of objects

boat = np.array([0, 0]) #boat position
boatVelocity = np.array([0.0, 0.0]) #this is our heading
boatMaxSpeed = 1 #change based on IRL performance
goal = np.array([0,0]) #goal position
wind = np.array([-1.7, -1]) #wind in x and y velocity (m/s)
windDir = wind / np.linalg.norm(wind) #normalize the wind vector


#timekeeping

dt = 1 #timestep for physics calculations
t0 = 0.0 #initial time 
tCurrent = 0.0 #current time


#information stored on the boat's current heading

onAB = False #whether the boat is currently trying to sail a course on either side of the no-go zone
onA = False 
onB = False





def physicsUpdate(): #update the world. If the wind is 

    global tCurrent, dt, boat, boatVelocity, wind, windDir, BG, goal

    tCurrent = tCurrent + dt
    
    print(tCurrent)

    boat = np.add(boat, boatVelocity*dt)
    
    #wind = [3*np.cos(tCurrent * (6.28/100) + np.pi/2), 3*np.sin(tCurrent * (6.28/100) + np.pi/2)]
    
    """
    BGraw = goal-boat #absolute distance to goal
    BG = BGraw / np.linalg.norm(BGraw) #convert to a unit vector
    
    wind = -BG
    """
    
    windDir = wind / np.linalg.norm(wind) #normalize the wind vector
    
    print(windDir)





def calcHeading(): #calculate the heading we should follow

    global tCurrent, dt, boat, boatVelocity, onAB, onA, onB

    #calculate BG (boat to goal vector)
    
    BGraw = goal-boat #absolute distance to goal
    BG = BGraw / np.linalg.norm(BGraw) #convert to a unit vector
    
    #compute the dot product of BG and windDir to see if we can sail directly towards the goal
    
    Dp = np.dot(BG, windDir)
    
    print("dot product of direction to goal and the wind is", end= ' ')
    print(Dp)
    
    
    if (Dp > Dpmin):
        #if current Dp is less than Dpmin, we can sail directly at the goal. Easy peasy!
        
        print("trying to sail directly at the goal")
        
        onAB = False #we are NOT sailing on the edge of the no-go zone if we sail directly at the goal
        return BG #return desired heading
        
        
    else:
        #if we can't sail directly at the goal we will have to decide how to tack
        #however, if we're already on an upwind course that's not directly at the goal, we'd like to continue on that course unless our crosstrack error is too high
        
        print("we cannot sail directly at the goal - calculating best heading")
        
        #checking if our crosstrack error is too high requires knowing the vectors A and B, so we'll start with that:
        #A and B are the vectors that lie on the edge of the no-go zone, so we'll just rotate the upwind direction by + and - theta, where theta is the minimum sailing angle
        
        #rotation matrix for minimum sailing angle:
            
        c, s = np.cos(minSailingAngle), np.sin(minSailingAngle)

        R = np.array(((c, -s), (s, c))) #rotation matrices
        R2 = np.array(((c, s), (-s, c))) 
        
        #multiply the matrices by the wind- MUST be the upwind direction! (which is just -windDir)
        
        A = np.matmul(-windDir, R)
        B = np.matmul(-windDir, R2)
        
        
        #now that we have A and B, we can find which points more in the direction we want to go
        ADBG = np.dot(A, BG) #dot product tells us which vector is pointing more towards the goal
        BDBG = np.dot(B, BG)
        

        if (onAB != True): #if we're not on a heading A or B but we aren't sailing directly at the goal, we need to start moving on A or B.
    
            if (ADBG > BDBG): #return whichever heading A or B points more towards the goal
                onAB = True
                onA = True
                onB = False
                return A
            else:
                onAB = True
                onA = False
                onB = True
                return B
            
            
                    
        else: #if we're on a heading A or B, we only want to change heading if we've accumulated too much crosstrack error
            
            cteThreshold = np.cos(minSailingAngle - np.arcsin(maxCte / np.linalg.norm(BGraw)))

            print("cteThreshold is", end= ' ')
            print(cteThreshold) 
            
            print("A dot BG is", end= ' ')  
            print(ADBG) 
            
            print("B dot BG is", end= ' ')   
            print(BDBG)               
                                      
            if (BDBG > cteThreshold):
                onAB = True
                onA = False
                onB = True
                return B
            
            if (ADBG > cteThreshold):
                onAB = True
                onA = True
                onB = False
                return A
            
            #if neither of the above statements evaluate to true we should just keep following whatever path we were already trying to follow
            
            if (onA):
                onAB = True
                onA = True
                onB = False
                return A
            if (onB):
                onAB = True
                onA = False
                onB = True
                return B
                
    print("you shouldn't be seeing this")
    return np.array(1,1)

    #I don't believe that there's a case that would cause this return statement to execute, but just in case I'm leaving this here.
        

                                  
xpos = []
ypos = []


def runSimulation():
    
    global boatVelocity, dt, xpos, ypos, windDir, goal, boat, BG, BGraw
    
    #we're doing it! woo hoo! boat go speed speed!
    
    
    while(1): #ruh roh raggy
        
        BGraw = goal-boat #absolute distance to goal
        BG = BGraw / np.linalg.norm(BGraw) #convert to a unit vector
        
        targetHeading = calcHeading()
        boatVelocity = targetHeading * boatMaxSpeed #the boat moves in the direction of the target heading
        
        physicsUpdate()
        
        #print(boat[0])
        #print(boat[1])
        
        xpos = np.append(xpos, boat[0])
        ypos = np.append(ypos, boat[1])
        
        plt.title("Sailing simulation") 
        plt.xlim([-300, 300])
        plt.ylim([-250, 250])
        plt.xlabel("x (meters)") 
        plt.ylabel("y (meters)") 
        plt.plot(xpos, ypos) #plot the boat's course 
        plt.plot(goal[0],goal[1],'ro') #plots goal location
        #plt.arrow(-100, 150, windDir[0]*100, windDir[1]*100)
        

        #plots and labels wind direction
        ax = plt.gca()
        ax.annotate("Wind", xy=[-100,100], xytext=[-100 + windDir[0]*100,100 + windDir[1]*100], arrowprops=dict(arrowstyle="<|-,head_width=0.1, head_length=0.1", edgecolor = 'k', facecolor = 'r', linewidth=1))

        plt.show()
        
        pause = input("click enter to continue, type exit to stop simulation ")
        if (pause == "exit"):
            break
        
        if (np.linalg.norm(BGraw) < 5): #if we're less than 5m from the goal we consider it reached
            break
        
        

#runSimulation ends if the goal is reached, so defining a new goal after it and running it again sees what the boat will do if it is asked to go to a new location



goal = np.array([-60, 60])
runSimulation()












                
        

                
                
        
        
        
        
        
        
        
    
    
        
        
        
        
    
    








