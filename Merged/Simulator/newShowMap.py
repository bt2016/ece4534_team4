"""
Shows the current map defined in coordinates.txt
"""

from globalVARS import *
from coordinateVisualizer import *
import matplotlib.pyplot as plt
import matplotlib.path as mpath
from matplotlib.path import Path
import matplotlib.patches as patches

#set the two axes of our map
plt.axes()
axes = plt.gca()
axes.set_xlim([1,90])
axes.set_ylim([1,90])

plt.grid(b=True, which='major', color='b')
plt.grid(b=True, which='minor', color='b')


#import map data from files
try:
   # receive list of coordinates from a file
   l_coordinates = list()
   prev_positions = list()
   token_data = list()
   rover_position = list()
   f = open("coordinates.txt", 'r')
   p = open("previous.txt", 'r')
   t = open("tokens.txt", 'r')
   f_coordinates = f.readlines()
   p_coordinates = p.readlines()
   t_coordinates = t.readlines()
   f.close()
   p.close()
   t.close()

   for coordinate in f_coordinates:

       if len(coordinate.split()) < 2:
          continue

       x = float(coordinate.split()[0])
       y = float(coordinate.split()[1])

       if len(coordinate.split()) == 4:
          isRover = coordinate.split()[2]
          direction = coordinate.split()[3]
          rover_position.append((x, y, isRover, direction)) 
       else:
          l_coordinates.append((x, y))

   for coordinate in p_coordinates:
       if len(coordinate.split()) == 2:
          xpos = float(coordinate.split()[0])
          ypos = float(coordinate.split()[1])
          prev_positions.append((xpos, ypos))

   for token in t_coordinates:
       xpos = float(token.split()[0])
       ypos = float(token.split()[1])
       found = token.split()[2] 
       token_data.append((xpos, ypos, found))

except Exception,e: print str(e)


#plot all the obstacles:
for obstacle in l_coordinates:
    circle = plt.Circle((obstacle[0], obstacle[1]), radius=5, fc='r')
    plt.gca().add_patch(circle)

#plot all the tokens if they've been found
for token in token_data:
    if token[2] == 'yes':
       circle = plt.Circle((token[0], token[1]), radius=1.5, fc='g')
       plt.gca().add_patch(circle)


#plot the rovers previous positions
xpositions = list()
ypositions = list()
for position in prev_positions:
    xpositions.append(position[0])
    ypositions.append(position[1])

plt.plot(xpositions, ypositions)





#plot the rover
if len(rover_position) == 1:
 
   orientation = rover_position[0][3]
   
   square = plt.Rectangle((rover_position[0][0] - 4, rover_position[0][1] - 4), 8, 8)
   plt.gca().add_patch(square)






plt.show()

