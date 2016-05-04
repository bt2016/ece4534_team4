
#sensorDisplay.py

#imports
import random,sys
import math
import matplotlib.pyplot as plt

def appendMap(message):
    try:
        r = ord(message[3])
        theta = ord(message[4])
        x = ord(message[5])
        y = ord(message[6])
        f = open("fullmap.txt", 'a')
        f.write(str.format("\n{0} {1} {2} {3}",r,theta,x,y))
        f.close()
    except:
        pass

def appendLines(message):
    try:
        r = ord(message[3])
        theta = ord(message[4])
        x = ord(message[5])
        y = ord(message[6])
        f = open("lines.txt", 'a')
        f.write(str.format("\n{0} {1} {2} {3}",r,theta,x,y))
        f.close()
    except:
        pass

def appendTargets(message):
    try:
        r = ord(message[3])
        theta = ord(message[4])
        x = ord(message[3])
        y = ord(message[4])
        lr = ord(message[5])
        std = ord(message[6])
        maxd = ord(message[7])
        f = open("targets.txt", 'a')
        f.write(str.format("\n{0} {1} {2} {3} {4} {5} {6}",r,theta,x,y,std,maxd,lr))
        f.close()
        print(str.format("appendTargets: (x,y)=({0},{1}) LR:{2}",x,y,lr))
    except:
        pass

def clearMap():
    print "request to clear fullmap.txt"
    f = open("fullmap.txt", 'w')
    f.write(str("0 0 0 0"))
    f.close()

def clearLines():
    print "request to clear lines.txt"
    f = open("lines.txt", 'w')
    f.write(str("0 0 0 0"))
    f.close()

def clearTargets():
    print "request to clear targets.txt"
    f = open("targets.txt", 'w')
    f.write(str("0 0 0 0 0 0 0"))
    f.close()

#Display the map for one sensor in polar coordinates
def displayFullMap():
    print "request to displayFullMap()"
    try:
        # receive list of coordinates from a file
        map_r = list()
        map_theta = list()
        lines_r = list()
        lines_theta = list()
        targets_r = list()
        targets_theta = list()

        f = open("fullmap.txt", 'r')
        f_coordinates = f.readlines()
        f.close()
        for coordinate in f_coordinates:
            r = int(float(coordinate.split()[0])) 
            theta = int(float(coordinate.split()[1]))
            map_r.append(r)
            map_theta.append(theta)

        f = open("lines.txt", 'r')
        f_coordinates = f.readlines()
        f.close()
        for coordinate in f_coordinates:
            r = int(float(coordinate.split()[0])) 
            theta = int(float(coordinate.split()[1]))
            lines_r.append(r)
            lines_theta.append(theta)

        f = open("targets.txt", 'r')
        f_coordinates = f.readlines()
        f.close()
        for coordinate in f_coordinates:
            r = int(float(coordinate.split()[0])) 
            theta = int(float(coordinate.split()[1]))
            targets_r.append(r)
            targets_theta.append(theta)

        #plt.close()

        #create the plot
        plot_map = plt.plot(map_theta, map_r, 'o')
        plot_lines = plt.plot(lines_theta, lines_r, 'o')
        plot_targets = plt.plot(targets_theta, targets_r, 'o')
        plt.axis([0, 90, 0, 150])
        plt.xlabel('angle (degrees)')
        plt.ylabel('distance (cm)')
        plt.title('Plot of Polar Coordinates from Sensor')
        plt.text(57, 142, 'raw data from sensor', fontsize=15, color='blue')
        plt.text(60, 135, 'targets from sensor', fontsize=15, color='green')
        plt.text(52, 128, 'targets from ProcessTask', fontsize=15, color='red')
        plt.grid(True)

        #add datapoint annotations
        for r,theta in zip(lines_r,lines_theta):
            if r != 0:
                plt.annotate(str.format('({0},{1})',theta,r), xy=(theta,r), xytext=(theta+5,r-15), color='green', arrowprops=dict(facecolor='green',shrink=0.2, frac=0.5))
        for r,theta in zip(targets_r, targets_theta):
            if r != 0:
                plt.annotate(str.format('({0},{1})',theta,r), xy=(theta,r), xytext=(theta+5,r-15), color='red', arrowprops=dict(facecolor='red',shrink=0.2, frac=0.5))

        #display the plot
        plt.show()
    except:
        import traceback
        print traceback.format_exc()

#Display the map for all sensors in Cartesian
def displayField():
    print "request to displayField()"
    try:
        # receive list of coordinates from a file
        map_x = list()
        map_y = list()
        lines_x = list()
        lines_y = list()
        targets_x = list()
        targets_y = list()
        leadrover = tuple()

        f = open("fullmap.txt", 'r')
        f_coordinates = f.readlines()
        f.close()
        for coordinate in f_coordinates:
            x = int(float(coordinate.split()[2])) 
            y = int(float(coordinate.split()[3]))
            map_x.append(x)
            map_y.append(y)

        f = open("lines.txt", 'r')
        f_coordinates = f.readlines()
        f.close()
        print("Lines:")
        for coordinate in f_coordinates:
            x = int(float(coordinate.split()[2])) 
            y = int(float(coordinate.split()[3]))
            lines_x.append(x)
            lines_y.append(y)
            print(str.format("    xy=({0},{1}) rtheta=({2},{3})",x,y,int(float(coordinate.split()[0])), int(float(coordinate.split()[1]))))

        f = open("targets.txt", 'r')
        f_coordinates = f.readlines()
        f.close()
        print("Targets:")
        for coordinate in f_coordinates:
            r = int(float(coordinate.split()[0]))
            theta = int(float(coordinate.split()[1]))
            x = int(float(coordinate.split()[2])) 
            y = int(float(coordinate.split()[3]))
            std = int(float(coordinate.split()[4]))
            maxd = int(float(coordinate.split()[5]))
            if (int(float(coordinate.split()[6])) == 1):
                leadrover = (x,y)
            targets_x.append(x)
            targets_y.append(y)
            print(str.format("    xy=({0},{1}) rtheta=({2},{3}) StandardDeviation={4} MaxDeviationSquared={5} LR:{6}",x,y,r,theta,std,maxd,int(float(coordinate.split()[6]))))

        #plt.close()

        #create the plot
        plt.plot([30,30],[0,90], linewidth=1, color='black')
        plt.plot([60,60],[0,90], linewidth=1, color='black')
        plt.plot([0,90],[30,30], linewidth=1, color='black')
        plt.plot([0,90],[60,60], linewidth=1, color='black')
        plot_map = plt.plot(map_x, map_y, 'o', color='blue')
        plot_targets = plt.plot(targets_x, targets_y, 'o', color='red', markersize=50)
        plot_lines = plt.plot(lines_x, lines_y, 'o', color='green')
        plt.axis([0, 90, 0, 90])
        plt.xlabel('distance (cm)')
        plt.ylabel('distance (cm)')
        plt.title('Plot of Cartesian Coordinates from Sensor Network')
        plt.text(57, 142, 'raw data from sensor', fontsize=15, color='blue')
        plt.text(60, 135, 'targets from sensor', fontsize=15, color='green')
        plt.text(52, 128, 'targets from ProcessTask', fontsize=15, color='red')
        plt.grid(True)

        #add LR annotation if it exists
        if leadrover != tuple():
            plt.plot(leadrover[0],leadrover[1], 'o', color='yellow')

        #add datapoint annotations
        '''
        for x,y in zip(lines_x,lines_y):
            if x != 0:
                plt.annotate(str.format('({0},{1})',x,y), xy=(x,y), xytext=(x+5,y-15), color='green', arrowprops=dict(facecolor='green',shrink=0.2, frac=0.5))
        '''
        for x,y in zip(targets_x, targets_y):
            if x != 0:
                plt.annotate(str.format('({0},{1})',x,y), xy=(x,y), xytext=(x+5,y-15), color='red', arrowprops=dict(facecolor='red',shrink=0.2, frac=0.5))

        #display the plot
        plt.show()
    except:
        import traceback
        print traceback.format_exc()

def echoCoordinate(message):
    #print message
    try:
        sensor_name = message[3]
        r = ord(message[4])
        theta = ord(message[5])
        if (ord(sensor_name) >= 97):
            print(str.format("Sensor{0}_LR: r= {1} theta= {2}",sensor_name,r,theta)) 
        else:
            print(str.format("Sensor{0}: r= {1} theta= {2}",sensor_name,r,theta)) 
    except:
        import traceback
        print traceback.format_exc()





#Display the map for all sensors in Cartesian
def displayAverage():
    print "request to displayAverage()"
    try:
        # receive list of coordinates from a file
        targets_x = list()
        targets_y = list()

        f = open("average.txt", 'r')
        f_coordinates = f.readlines()
        f.close()
        print("Average Targets:")
        for coordinate in f_coordinates:
            x = int(float(coordinate.split()[0])) 
            y = int(float(coordinate.split()[1]))
            std_x = int(float(coordinate.split()[2]))
            std_y = int(float(coordinate.split()[3]))
            targets_x.append(x)
            targets_y.append(y)
            print(str.format("    xy=({0},{1}) std_x={2} std_y={3}",x,y,std_x,std_y))

        #create the plot
        plt.plot([30,30],[0,90], linewidth=1, color='black')
        plt.plot([60,60],[0,90], linewidth=1, color='black')
        plt.plot([0,90],[30,30], linewidth=1, color='black')
        plt.plot([0,90],[60,60], linewidth=1, color='black')
        plot_targets = plt.plot(targets_x, targets_y, 'o', color='red')
        plt.axis([0, 90, 0, 90])
        plt.xlabel('distance (cm)')
        plt.ylabel('distance (cm)')
        plt.title('Plot of Average Cartesian Coordinates from Sensor Network')
        plt.grid(True)

        #display the plot
        plt.show()
    except:
        import traceback
        print traceback.format_exc()





