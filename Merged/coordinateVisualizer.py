

# coordinateVisualizer.py
# 
# This program reads a list of coordinates from a text file that is specified by a
#     command line argument. Specify the encoding, rectangular or polar by using
#     a command line argument. The coordinates should be space delimited
# The program will create a  90cmx90cm map on the terminal screen where each character
#     location corresponds to a 1cm^2 area.
# Gridlines will appear every 30cm in both x and y directions.
# Targets will be identified as "X"
# The origin is in the lower left-hand corner, and the first row/column is (0,0)
#     where x,y are in rectangular coordinates and in cm
#
#
#


# Imports
import random,sys
import math
import matplotlib.pyplot as plt
from globalVARS import *
from random import randint

from msgToCoordinator import processMsgToCoordinator as sendToCoordinator



#restricts the range of n 
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


def appendRectFromPolar(message, fromCORD=False):
    #print(message)
    r = ord(message[3])
    theta = ord(message[4])
    xpos = r  #int(r*math.cos(theta*(math.pi/180)))
    ypos = theta  #int(r*math.sin(theta*(math.pi/180)))
    #print(str.format("r={0} theta={1}",r,theta))
    #print(str.format("xpos={0} ypos={1}",xpos,ypos))

    if not fromCORD:
       f = open("coordinates.txt", 'a')
    else:
       f = open("from_coordinator.txt", 'a')

    f.write(str.format("\n{0} {1}",xpos,ypos))
    f.close()


#This function appends a set of polar coordinates from a message to coordinates.txt
#it will ignore all information in the obstacle other than midpoint_r and midpoint_theta
def coordinatorAppendTargets(message):
    try:
        #print(message)
        x = ord(message[3])
        y = ord(message[4])
        LR = ord(message[5])

        f = open("coordinates.txt", 'a')

        if LR: 
           f.write("\n{0} {1} {2}".format(x, y, "yes unknown"))
        else:
           f.write("\n{0} {1}".format(x, y))

        f.close()


    except:
        pass

def echoCoordinate(message):
    try:
        r = ord(message[3])
        theta = ord(message[4])
        xpos = ord(message[5])
        ypos = ord(message[6])
        slope = ord(message[7])
        arclength = ord(message[8])
        print(str.format("midpoint_r= {0} midpoint_theta= {1} midpoint_x= {2} midpoint_y= {3} slope= {4} length_of_arc= {5}",r,theta,xpos,ypos,slope,arclength))
    except:
        pass

#This function clears the text file coordinates.txt
def clearListOfCoordinates():
    f = open("coordinates.txt", 'w')
    f.close()




#new_direction is a string that is either up, right, left, or down
def turn(l_or_r, amt):
    try:
        # receive list of coordinates from a file
        l_coordinates = list()
        rover_position = {}
        f = open("coordinates.txt", 'r')
        f_coordinates = f.readlines()
        f.close()
        for coordinate in f_coordinates:
            xpos = float(coordinate.split()[0])
            ypos = float(coordinate.split()[1])

            l_coordinates.append((xpos, ypos))

            if len(coordinate.split()) == 4:
               isRover = coordinate.split()[2]
               direction = coordinate.split()[3]
               rover_position[(xpos, ypos)] = [isRover, direction]


        f = open("coordinates.txt", 'w')
        for index, line in enumerate(l_coordinates):
            if line in rover_position:
               xpos = line[0]
               ypos = line[1]
               direction = int(rover_position[line][1])

               if(l_or_r == "LEFT"):
                  direction = direction + amt
               elif(l_or_r == "RIGHT"):
                  direction = direction - amt
 
               direction = max(0, direction)
               direction = min(360, direction)    
   
               if index == 0:
                  f.write(str("{0} {1} {2} {3}".format(xpos, ypos, rover_position[line][0], direction)))

               else:
                  f.write(str("\n{0} {1} {2} {3}".format(xpos, ypos, rover_position[line][0], direction)))

            else:
               if index == 0:
                  f.write(str("{0} {1}".format(line[0], line[1])))
               else:
                  f.write(str("\n{0} {1}".format(line[0], line[1])))

        f.close()

    except Exception,e: print str(e)





def moveForward(MOVE_AMT):
    try:
        # receive list of coordinates from a file
        l_coordinates = list()
        token_data = list()
        rover_position = {}
        f = open("coordinates.txt", 'r')
        p = open("previous.txt", 'a')
        t = open("tokens.txt", 'r')
        f_coordinates = f.readlines()
        t_coordinates = t.readlines()
        f.close()
        t.close()
        for coordinate in f_coordinates:
            xpos = float(coordinate.split()[0]) 
            ypos = float(coordinate.split()[1])

            l_coordinates.append((xpos, ypos))
  
            if len(coordinate.split()) == 4:
               isRover = coordinate.split()[2]
               direction = coordinate.split()[3]
               rover_position[(xpos, ypos)] = [isRover, direction]

        for coordinate in t_coordinates:
            xpos = float(coordinate.split()[0]) 
            ypos = float(coordinate.split()[1])
            found = coordinate.split()[2] #string, yes or no

            token_data.append((xpos, ypos, found))


        f = open("coordinates.txt", 'w')
        t = open("tokens.txt", 'w')
        for index, line in enumerate(l_coordinates):
            if line in rover_position:
               
               p.write(str("\n{0} {1}".format(line[0], line[1])))


               xpos = line[0]
               ypos = line[1] 
               x1 = xpos
               y1 = ypos

               direction = int(rover_position[line][1])
               direction_radians = math.radians(direction)

               """
               Direction needs to be an integer and we need to calculate the new position
               """ 

               ypos = int(ypos + (MOVE_AMT * math.sin(direction_radians))) 
               xpos = int(xpos + (MOVE_AMT * math.cos(direction_radians))) 

               ypos = min(90, ypos)
               ypos = max(1, ypos)
               xpos = min(90, xpos)
               xpos = max(1, xpos)

               """
               #introduce error into the system
               if MOVE_AMT > 2: 
                  flux1 = randint(-ERROR_AMT, ERROR_AMT)              
                  flux2 = randint(-ERROR_AMT, ERROR_AMT)              

                  xpos = clamp(flux1+xpos, 1, 90)
                  ypos = clamp(flux2+ypos, 1, 90)
               """
               #figure out if we hit any tokens
               for index2, token in enumerate(token_data):
                   x2 = xpos
                   y2 = ypos
                   tx = token[0]
                   ty = token[1]
                   found = token[2]

                   if found == 'no':
                      numerator = abs( ((y2-y1)*tx) - ((x2-x1)*ty) + (x2*y1) - (y2*x1)  )
                      denom = math.sqrt( abs(((y2-y1)*(y2-y1))) + abs(((x2-x1)*(x2-x1))) )
                      distance = numerator / float(denom)
                
                      if distance <= ROVER_WIDTH:
                         found = 'yes' 
                         #we need to send the coordinator a message
                         DATA_MESSAGE = '~{0}2oooooo.'.format(TYPEC_TOKEN_FOUND)
                         sendToCoordinator(DATA_MESSAGE)                         
                         print("TOKEN FOUND!")
       
                   if index2 == 0:
                      t.write("{0} {1} {2}".format(tx, ty, found))
                   else:
                      t.write("\n{0} {1} {2}".format(tx, ty, found))
                      


               if index == 0:
                  f.write(str("{0} {1} {2} {3}".format(xpos, ypos, rover_position[line][0], rover_position[line][1])))

               else:
                  f.write(str("\n{0} {1} {2} {3}".format(xpos, ypos, rover_position[line][0], rover_position[line][1])))
         
            else:
               if index == 0:
                  f.write(str("{0} {1}".format(line[0], line[1])))
               else:
                  f.write(str("\n{0} {1}".format(line[0], line[1])))

        #send a message saying the move is complete 
        DATA_MESSAGE = '~{0}2oooooo.'.format(TYPEC_LR_MOVE_COMPLETE) 
        #sendToCoordinator(DATA_MESSAGE)

        f.close()
        p.close()

    except Exception,e: print str(e)




#This function displays the map with cartesian axes and coordinate points
def runCoordinateVisualizer():
    try:
        # receive list of coordinates from a file
        l_coordinates = list()
        polar_coordinates = list()
        f = open("coordinates.txt", 'r')
        f_coordinates = f.readlines()
        f.close()
        for coordinate in f_coordinates:
            r = float(coordinate.split()[0]) 
            theta = float(coordinate.split()[1])
            xpos = int(r*math.cos(theta*(math.pi/180)))
            ypos = int(r*math.sin(theta*(math.pi/180)))
            l_coordinates.append((xpos,ypos))
            polar_coordinates.append((r,theta))
        
        # print to console
        print("Cartesian Coordinates:")
        print(l_coordinates)
        print(str.format("      {0}{0}{0}{0}{0}{0}{0}{0}{0} ","__________"))
        for y in reversed(range(0,90)):
            rowstr = str()
            if ((y%10 == 0) and (y!=0)):
                rowstr += str.format("{0}cm |",y) #axis label for horizontal gridline
            else:
                rowstr += "     |"

            for x in range(0,90):
                try:
                    l_coordinates.index((x,y))
                    rowstr += "X"
                except ValueError:
                    if ((y%30 == 0) and (y!=0)): #check to add a horizontal gridline
                        rowstr += "-"
                    elif ((x%30 == 0) and (x!=0)): #check to add a vertical gridline
                        rowstr += "|"
                    else:
                        rowstr += " "
            rowstr += "|"
            print(rowstr)
        print(str.format("     |{0}{0}{0}{0}{0}{0}{0}{0}{0}|","----------"))

        #axis labels for vertical gridlines
        rowstr = "     "
        for x in range(0,90):
            if (x%30 == 0):
                if (x<10):
                    rowstr += str.format("|{0}cm   ",x) #x axis label for 0cm
                else:
                    rowstr = rowstr[:-5]
                    rowstr += str.format("|{0}cm ",x) #add x axis labels
            else:
                rowstr += " "
        print(rowstr)
            
            
    except Exception,e: print str(e)

def plotPolarCoordinates():
    try:
        # receive list of coordinates from a file
        r_coordinates = list()
        theta_coordinates = list()
        #f = open("coordinates.txt", 'r')
        f = open("coordinates.txt", 'r')
        f_coordinates = f.readlines()
        f.close()
        for coordinate in f_coordinates:
            r = int(float(coordinate.split()[0])) 
            theta = int(float(coordinate.split()[1]))
            r_coordinates.append(theta)
        #plt.ion()
        plt.close()
        plt.plot(theta_coordinates, r_coordinates, 'o')
        plt.axis([0, 90, 0, 150])
        plt.xlabel('angle (degrees)')
        plt.ylabel('distance (cm)')
        plt.title('Plot of Polar Coordinates from Sensor')
        plt.grid(True)
        plt.show()
    except:
        pass

#This function displays the map with polar axes and coordinate points
def runCoordinateVisualizer_Polar():
    try:
        # receive list of coordinates from a file
        polar_coordinates = list()
        f = open("coordinates.txt", 'r')
        f_coordinates = f.readlines()
        f.close()
        for coordinate in f_coordinates:
            r = int(float(coordinate.split()[0])) 
            theta = int(float(coordinate.split()[1]))
            polar_coordinates.append((theta,r))
        
        # print to console
        print("Polar Coordinates (r[cm] theta[deg]):")
        #print(polar_coordinates)
        for t in polar_coordinates:
            print(str.format("{0} {1}",t[0],t[1]))
        print(str.format("      {0}{0}{0}{0}{0}{0}{0}{0}{0} ","__________"))

        for y in reversed(range(0,120)):
            rowstr = str()
            if ((y%10 == 0) and (y!=0)):
                rowstr += str.format("{0}cm |",y) #axis label for horizontal gridline
            else:
                rowstr += "     |"

            for x in range(0,90):
                try:
                    polar_coordinates.index((x,y))
                    #l_coordinates.index((x,y))
                    rowstr += "X"
                except ValueError:
                    if ((y%30 == 0) and (y!=0)): #check to add a horizontal gridline
                        rowstr += "-"
                    elif ((x%30 == 0) and (x!=0)): #check to add a vertical gridline
                        rowstr += "|"
                    else:
                        rowstr += " "
            rowstr += "|"
            print(rowstr)
        print(str.format("     |{0}{0}{0}{0}{0}{0}{0}{0}{0}|","----------"))

        #axis labels for vertical gridlines
        rowstr = "     "
        for x in range(0,90):
            if (x%30 == 0):
                if (x<10):
                    rowstr += str.format("|{0}deg  ",x) #x axis label for 0cm
                else:
                    rowstr = rowstr[:-5]
                    rowstr += str.format("|{0}deg",x) #add x axis labels
            else:
                rowstr += " "
        print(rowstr)
            
            
    except Exception,e: print str(e)



