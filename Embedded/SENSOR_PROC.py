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
   else:
      sock.sendto(MESSAGE, (COORDINATOR_IP, COORDINATOR_PORT))
      

def processMsgFromBrooke(type, MESSAGE):
    if type == TYPE_BROOKE_APPENDPOLAR:
        appendRectFromPolar(MESSAGE)
    elif type == TYPE_BROOKE_DISPLAY:
        runCoordinateVisualizer()
    elif type == TYPE_BROOKE_CLEAR:
        clearListOfCoordinates()










