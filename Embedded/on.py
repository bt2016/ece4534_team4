"""
Test sending a message over UDP to a WiFly
"""

import socket

UDP_IP = "192.168.42.41"
UDP_PORT = 10000
#MESSAGE = "Embedded is so much fun! (Sarcasm)\r\n"
MESSAGE = '~roo.'

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))


print("Message successfully sent")
