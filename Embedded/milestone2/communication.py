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
recver_addr = '192.168.42.29'
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


#Empty message buffer with corresponding function to fill and send it.
#Eventually we should make this into a class with functions so we can handle multiple 
#connections, not just one.    
message_buffer = ""  # ~roo.
def leBuffer(byte):
    global message_buffer
    #start byte received
    if(byte == "~"):
        message_buffer = "~"       
    
    elif(byte == "."):
        message_buffer = message_buffer + byte
        
        if( len(message_buffer) == 5 ):
            udpSend(recver_addr, recver_port, message_buffer)
            message_buffer = ""



#Main loop
while True:

    print("Blocking while trying to connect to sender")
    sender_connection, client_address = sender_sock.accept()
    #sender_connection.settimeout(0.01) #allows this socket to block for 10ms only 
    
     
    #Receive the hello
    data = sender_connection.recv(6)        

    # Receive the data in small chunks and retransmit it
    while True:
        
        data = sender_connection.recv(1)
            
        #print >>sys.stderr, 'received "%s"' % data
        #print >>sys.stderr, '"%s"' % data
        
        leBuffer(data)
            
        
