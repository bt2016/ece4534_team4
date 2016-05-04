"""
The sensor sent a message. Call the below function
for to handle processing the message that the sensor pic has sent
It will process the data and send it off to whoever needs it. 
"""

from globalVARS import *
#from coordinateVisualizer import *
from sensorDisplay import *
import socket
import time
from coordinateVisualizer import *

def processMsgToCoordinator(MESSAGE):

    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.sendto(MESSAGE, (COORDINATOR_IP, COORDINATOR_PORT))

    #print("Message successfully sent")

def processMsgToSensor(MESSAGE):
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.sendto(MESSAGE, (SENSOR_IP, SENSOR_PORT))



def processMsgFromCoordinator(type, MESSAGE):
   
   print("Message received from COORDINATOR")
   print("RAW COORDINATOR MSG: {0}".format(MESSAGE))    
   print(" ".join(hex(ord(n)) for n in MESSAGE))

   sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP

   if type == TYPEC_LR_SENSOR_TO_FR:
      sock.sendto(MESSAGE, (FOLLOWER_IP, FOLLOWER_PORT))

   elif type == TYPEC_MOTOR_CTRL or type == TYPEC_LR_HANDSHAKE:
      sock.sendto(MESSAGE, (LR_IP, LR_PORT))


   elif type == TYPEC_TKN_REQUEST:
      MESSAGE = '{0}{1}xddddd{2}{3}'.format(MESSAGE_START_BYTE, TYPEC_TKN_REQUEST, chr(NUMBER_OF_TOKENS), MESSAGE_STOP_BYTE)
      sock.sendto(MESSAGE, (COORDINATOR_IP, COORDINATOR_PORT))
      
      coordinator_token_number_sent = True
      print("Updating the token type on the coordinator")


   elif type == TYPEC_ACK_TOKEN: 
      print("The coordinator has acknowleged it has received the number of tokens")
      coordinator_ack_token_number = True

   elif type == TYPEC_CLEAR_MAP:
        clearListOfCoordinates(fromCORD=True)

   elif type == TYPEC_MAP_DATA:
        appendRectFromPolar(MESSAGE, fromCORD=True)
        print("Appending this msg to coord.txt: {0}".format(MESSAGE))

   elif type == TYPEC_UPDATE_MAP: 
        #update the coordinates on the coordinator 
        time.sleep(0.5) #wait a half second          
        """
        f = open("coordinates.txt", 'r')
        f_coordinates = f.readlines()


        for coordinate in f_coordinates:
            xpos = float(coordinate.split()[0]) 
            ypos = float(coordinate.split()[1])

            roverchar = "N"
            if len(coordinate.split()) == 4:
               roverchar = "Y"

            DATA_MESSAGE = '~{0}2{1}{2}{3}ooo.'.format(TYPEC_MAP_DATA, chr(int(xpos)), chr(int(ypos)), chr(ord(roverchar)))
            processMsgToCoordinator(DATA_MESSAGE)        
            print("Sent x: {0}, y: {1} rover: {2} ".format(xpos, ypos, roverchar))
            time.sleep(0.7) #sleep for 700 ms 
        """
        #DATA_MESSAGE = '~{0}2oooooo.'.format(TYPEC_END_MAP_DATA)
        #processMsgToSensor(MESSAGE)        
        print("Coordinator used an old message type. IGNORING")
        #clear the local list of coordinates:


   elif type == TYPE_SENSOR_SINGLEREQUESTED:
        time.sleep(0.5) #wait a half second          
        #processMsgToSensor(MESSAGE)        
        #clearListOfCoordinates()
        print("Coordinator requested SINGLE UPDATE with store bit set to: {0}".format(ord(MESSAGE[3])))

   elif type == TYPE_SENSOR_MULTIPLEREQUESTED:
        #processMsgToSensor(MESSAGE)        
        #clearListOfCoordinates()
        print("Coordinator requested MULTIPLE UPDATE with store bit set to: {0}".format(ord(MESSAGE[4])))

        f = open("coordinates.txt", 'r')
        f_coordinates = f.readlines()
        f.close() 

        time.sleep(0.5)
 
        for coordinate in f_coordinates:

            if(len(coordinate.split()) < 2):
               continue

            xpos = float(coordinate.split()[0])
            ypos = float(coordinate.split()[1])

            roverchar = "N"
            if len(coordinate.split()) == 4:
               roverchar = "Y"

            DATA_MESSAGE = '~{0}2{1}{2}{3}ooo.'.format(TYPE_SENSOR_APPENDTARGETS, chr(int(xpos)), chr(int(ypos)), chr(ord(roverchar)))
            processMsgToCoordinator(DATA_MESSAGE)
            print("RPI Sent (to coordinator) x: {0}, y: {1} rover: {2} ".format(xpos, ypos, roverchar))
            time.sleep(0.7) #sleep for 700 ms 

        DATA_MESSAGE = '~{0}2oooooo.'.format(TYPE_SENSOR_ENDUPDATE)
        processMsgToCoordinator(DATA_MESSAGE)

        print('End of sensor update (msg sent)')





   elif type == TYPEC_MOVE_FORWARD:
        amt = int(hex(ord(MESSAGE[3])), 0)
        moveForward(amt)          
        print("The LR has been instructed to move forwards by {0} cm's".format(amt))
        #DATA_MESSAGE = '~{0}2oooooo.'.format(TYPEC_LR_MOVE_COMPLETE)
        #processMsgToCoordinator(DATA_MESSAGE)        
         
 
   elif type == TYPEC_ROTATE:
        #take the message apart
        #left of right will be encoded in MESSAGE[3] as left being 0, right being 1 WRT the origin of the field
        #the amount of degrees will be encoded in MESSAGE[4] and will always be positive. 
      
        direction = ""
        if( int(hex(ord(MESSAGE[3])), 0) ):
            direction = "RIGHT"
        else:
            direction = "LEFT"

        amt1 = int(hex(ord(MESSAGE[4])), 0)
        amt2 = int(hex(ord(MESSAGE[5])), 0)
        amt = amt1 + amt2

        turn(direction, amt)
            
        print("The LR has been instructed to turn to the {0} by {1} degrees".format(direction, amt))



   #The below types are outdated...
   elif type == TYPEC_TURN_LEFT:
        turn("left")
        print("The LR has been instructed to turn left")

   elif type == TYPEC_TURN_UP:
        turn("up")
        print("The LR has been instructed to turn up")

   elif type == TYPEC_TURN_DOWN:
        turn("down")
        print("The LR has been instructed to turn down")

   elif type == TYPEC_TURN_RIGHT:
        turn("right")
        print("The LR has been instructed to turn right")

 
 


   else:
      sock.sendto(MESSAGE, (COORDINATOR_IP, COORDINATOR_PORT))
 


def processMsgFromBrooke(type, MESSAGE):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(str.format("Brooke sent a message of type {0}",type))
    if type == TYPE_SENSOR_UPDATEREQUESTED:
        sock.sendto(MESSAGE, (SENSOR_IP, SENSOR_PORT))
    elif type == TYPE_SENSOR_SINGLEREQUESTED:
        sock.sendto(MESSAGE, (SENSOR_IP, SENSOR_PORT))
    elif type == TYPE_SENSOR_MULTIPLEREQUESTED:
        sock.sendto(MESSAGE, (SENSOR_IP, SENSOR_PORT))


def processMsgFromSensor(type, MESSAGE):

    #print("RAW SENSOR MSG: {0}".format(MESSAGE))    
    print("Message received from SENSOR")

    if type == TYPE_SENSOR_APPENDMAP:
        appendMap(MESSAGE)
    elif type == TYPE_SENSOR_APPENDLINES:
        appendLines(MESSAGE)
    elif type == TYPE_SENSOR_APPENDTARGETS:
        appendTargets(MESSAGE)
        
        #do something with this message on the pi side to update the pi's values
        #also, send this message to the coordinator
        coordinatorAppendTargets(MESSAGE)
        #processMsgToCoordinator(MESSAGE)        
        print("SENSOR has sent a target with data x: {0}, y: {1}, LR?: {2}".format(ord(MESSAGE[3]), ord(MESSAGE[4]), ord(MESSAGE[5])))


    elif type == TYPE_SENSOR_CLEARMAP:
        clearMap()

    elif type == TYPE_SENSOR_CLEARLINES:
        clearLines()

    elif type == TYPE_SENSOR_CLEARTARGETS:
        clearTargets()

    elif type == TYPE_SENSOR_CLEARALL:
        clearMap()
        clearLines()
        clearTargets()

    elif type == TYPE_SENSOR_DISPLAYFULLMAP:
        displayFullMap()

    elif type == TYPE_SENSOR_DISPLAYFIELD:
        displayField()

    elif type == TYPE_SENSOR_ECHO:
        echoCoordinate(MESSAGE)
        

    elif type == TYPE_SENSOR_ENDUPDATE:
        #send this message to the coordinator too
        """
        Another way to do this would be to send the map to the coordinator right here by reading in the 
        now populated coordinates.txt file which would have data appended to it from APPENDTARGETS. 
        """
        print("The sensor sent an ENDUPDATE request")
        f = open("coordinates.txt", 'r')
        f_coordinates = f.readlines()
        f.close()

        for coordinate in f_coordinates:

            if(len(coordinate.split()) < 2): 
               continue

            xpos = float(coordinate.split()[0]) 
            ypos = float(coordinate.split()[1])

            roverchar = "N"
            if len(coordinate.split()) == 4:
               roverchar = "Y"

            DATA_MESSAGE = '~{0}2{1}{2}{3}ooo.'.format(TYPE_SENSOR_APPENDTARGETS, chr(int(xpos)), chr(int(ypos)), chr(ord(roverchar)))
            processMsgToCoordinator(DATA_MESSAGE)        
            print("RPI Sent (to coordinator) x: {0}, y: {1} rover: {2} ".format(xpos, ypos, roverchar))
            time.sleep(0.7) #sleep for 700 ms 
        DATA_MESSAGE = '~{0}2oooooo.'.format(TYPE_SENSOR_ENDUPDATE)
        processMsgToCoordinator(DATA_MESSAGE)        
         
        print('End of sensor update') 







