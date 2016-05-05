import socket
import sys


print("Creating a socket for the sender")
sender_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#Bind sock to the port
sender_address = ('192.168.42.1', 10000)
sender_sock.bind(sender_address)


# Listen for incoming connections. 
#The 1 is the number of backlog of connections that can queue (1 in this case)
sender_sock.listen(1)


"""
Receiver options
"""
recver_addr = '192.168.42.1'
recver_port = 10001


print("Initialization complete")



def udpSend(UDP_IP, UDP_PORT, MESSAGE):
    print "UDP target IP:", UDP_IP
    print "UDP target port:", UDP_PORT
    print "message:", MESSAGE

    sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    print("Message successfully sent")


#Main loop
while True:

    print("Blocking while trying to connect to sender")
    sender_connection, client_address = sender_sock.accept()
    
    
        # Receive the data in small chunks and retransmit it
        
           # data = connection.recv(1)
    sender_connection.send('n') 
    sender_connection.close() 
           #udpSend(recver_addr, recver_port, 'n')
