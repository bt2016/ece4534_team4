


import socket
from globalVARS import *

UDP_IP = LR_IP
UDP_PORT = LR_PORT
MESSAGE = '~{0}2oooooo.'.format(TYPEC_MOTOR_CTRL)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

print("Message Sent")
