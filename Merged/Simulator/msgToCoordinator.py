
import socket
from globalVARS import *

def processMsgToCoordinator(MESSAGE):

    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.sendto(MESSAGE, (COORDINATOR_IP, COORDINATOR_PORT))

    #print("Message successfully sent")
