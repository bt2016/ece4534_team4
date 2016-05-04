"""
Test sending a message over UDP to a WiFly
"""

import socket
from globalVARS import *
import time

UDP_IP = COORDINATOR_IP
UDP_PORT = COORDINATOR_PORT
MESSAGE = '~{0}2oooooo.'.format(TYPEC_MAKE_MOVE)

#print "UDP target IP:", UDP_IP
#print "UDP target port:", UDP_PORT
#print "message:", "MAKE MOVE"

moves = 520

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

for move in range(0, moves):
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    time.sleep(0.1) #last value was 0.2

#print("Message successfully sent")
