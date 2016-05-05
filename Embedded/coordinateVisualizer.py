

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

<<<<<<< HEAD
def appendRectFromPolar(message):
    try:
        #print(message)
        r = ord(message[3])
        theta = ord(message[4])
        #xpos = int(r*math.cos(theta*(math.pi/180)))
        #ypos = int(r*math.sin(theta*(math.pi/180)))
        #print(str.format("r={0} theta={1}",r,theta))
        #print(str.format("xpos={0} ypos={1}",xpos,ypos))
        f = open("coordinates.txt", 'a')
        f.write(str.format("\n{0} {1}",r,theta))
        f.close()
    except:
        pass
=======
>>>>>>> af8f215e460b092323b529c68f9ea0fdf41604a5

#This function appends a set of polar coordinates from a message to coordinates.txt
#it will ignore all information in the obstacle other than midpoint_r and midpoint_theta
def appendPolar(message):
    try:
        #print(message)
        r = ord(message[3])
        theta = ord(message[4])
        #print(str.format("r={0} theta={1}",r,theta))
        f = open("coordinates.txt", 'a')
        f.write(str.format("\n{0} {1}",r,theta))
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
    f.write(str("0 0"))
    f.close()


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



