

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

def appendRectFromPolar(message):
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

def clearListOfCoordinates():
    f = open("coordinates.txt", 'w')
    f.write(str("0 0"))
    f.close()

def runCoordinateVisualizer():
    try:
        # receive list of coordinates from a file
        l_coordinates = list()
        f = open("coordinates.txt", 'r')
        f_coordinates = f.readlines()
        f.close()
        for coordinate in f_coordinates:
            r = float(coordinate.split()[0]) 
            theta = float(coordinate.split()[1])
            xpos = int(r*math.cos(theta*(math.pi/180)))
            ypos = int(r*math.sin(theta*(math.pi/180)))
            l_coordinates.append((xpos,ypos))
        
        # print to console
        print("Cartesian Coordinates:")
        print(l_coordinates)
        print(str.format("      {0}{0}{0}{0}{0}{0}{0}{0}{0} ","__________"))

        for y in reversed(range(0,90)):
            rowstr = str()
            if ((y%30 == 0) and (y!=0)):
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




