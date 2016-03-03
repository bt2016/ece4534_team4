"""
The sensor sent a message. Call the below function
for to handle processing the message that the sensor pic has sent
It will process the data and send it off to whoever needs it. 
"""

from globalVARS import *
from coordinateVisualizer import *
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
      sock.sendto(MESSAGE, (COORDINATOR_IP, COORDINATOR_PORT))
      



def processMsgFromBrooke(type, MESSAGE):
    if type == TYPE_BROOKE_APPENDPOLAR:
        appendRectFromPolar(MESSAGE)
    elif type == TYPE_BROOKE_DISPLAY:
        runCoordinateVisualizer()
    elif type == TYPE_BROOKE_CLEAR:
        clearListOfCoordinates()










