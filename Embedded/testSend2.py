"""
Test sending a message over UDP to a WiFly
"""

import socket
from globalVARS import * 

UDP_IP = LR_IP
UDP_PORT = LR_PORT
MESSAGE = '~o2oooooo.'

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))


print("Message successfully sent")
