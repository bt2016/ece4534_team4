"""
The sensor sent a message. Call the below function
for to handle processing the message that the sensor pic has sent
It will process the data and send it off to whoever needs it. 
"""

from globalVARS import *
#from coordinateVisualizer import *
from sensorDisplay import *
import socket

def processMsgToCoordinator(MESSAGE):

    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.sendto(MESSAGE, (COORDINATOR_IP, COORDINATOR_PORT))

    #print("Message successfully sent")


def processMsgFromCoordinator(type, MESSAGE):

   #print(" ".join(hex(ord(n)) for n in MESSAGE))

   sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP

   if type == TYPEC_LR_SENSOR_TO_FR:
      sock.sendto(MESSAGE, (FOLLOWER_IP, FOLLOWER_PORT))

   elif type == TYPEC_MOTOR_CTRL or type == TYPEC_LR_HANDSHAKE:
      sock.sendto(MESSAGE, (LR_IP, LR_PORT))


   elif type == TYPEC_TKN_REQUEST:
      MESSAGE = '{0}{1}xddddd{2}{3}'.format(MESSAGE_START_BYTE, TYPEC_TKN_REQUEST, chr(NUMBER_OF_TOKENS), MESSAGE_STOP_BYTE)
<<<<<<< HEAD
      sock.sendto(MESSAGE, (COORDINATOR_IP, COORDINATOR_PORT))
      
      coordinator_token_number_sent = True
      print("Updating the token type on the coordinator")


   elif type == TYPEC_ACK_TOKEN: 
      print("The coordinator has acknowleged it has received the number of tokens")
      coordinator_ack_token_number = True

   elif type == TYPEC_CLEAR_MAP:
        clearListOfCoordinates()

   elif type == TYPEC_MAP_DATA:
        appendRectFromPolar(MESSAGE)
        print("Appending this msg to coord.txt: {0}".format(MESSAGE))

   elif type == TYPEC_UPDATE_MAP: 
        runCoordinateVisualizer()

   else:
=======
>>>>>>> af8f215e460b092323b529c68f9ea0fdf41604a5
      sock.sendto(MESSAGE, (COORDINATOR_IP, COORDINATOR_PORT))
      
      coordinator_token_number_sent = True
      print("Updating the token type on the coordinator")


<<<<<<< HEAD

def processMsgFromBrooke(type, MESSAGE):
    #print(str.format("Brooke sent a message of type {0}",type))
    if type == TYPE_BROOKE_APPENDPOLAR:
=======
   elif type == TYPEC_ACK_TOKEN: 
      print("The coordinator has acknowleged it has received the number of tokens")
      coordinator_ack_token_number = True

   elif type == TYPEC_CLEAR_MAP:
        clearListOfCoordinates()

   elif type == TYPEC_MAP_DATA:
>>>>>>> af8f215e460b092323b529c68f9ea0fdf41604a5
        appendRectFromPolar(MESSAGE)
        print("Appending this msg to coord.txt: {0}".format(MESSAGE))

   elif type == TYPEC_UPDATE_MAP: 
        runCoordinateVisualizer()

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
    elif type == TYPE_SENSOR_MULTIPLEREQUESTED:
        sock.sendto(MESSAGE, (SENSOR_IP, SENSOR_PORT))
    elif type == TYPE_SENSOR_ACK:
        sock.sendto(MESSAGE, (SENSOR_IP, SENSOR_PORT))


def processMsgFromSensor(type, MESSAGE):
    #print MESSAGE
    if type == TYPE_SENSOR_APPENDMAP:
        appendMap(MESSAGE)
    elif type == TYPE_SENSOR_APPENDLINES:
        appendLines(MESSAGE)
    elif type == TYPE_SENSOR_APPENDTARGETS:
        appendTargets(MESSAGE)
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
        print('End of sensor update')












