import socket
import threading
import SocketServer as socketserver

#for controlled close:
import sys
import signal
import time

#import all our variables
from globalVARS import *

from SENSOR_PROC import processMsg as SENSORprocessMsg

class ThreadedTCPRequestHandler(socketserver.BaseRequestHandler):

    typesAndBytes = {}
    #total_count = 0

    def trackIncoming(self, type, num):
  
        #its in our dictionary, check it
        if self.typesAndBytes.has_key(type):
          
           last_msg_num = self.typesAndBytes[type]

           if last_msg_num == 255: 
              last_msg_num = -1

           #if (last_msg_num + 1) != num:
              #print("Type {0} skipped message. I expected #{1} but received #{2}".format(type, (last_msg_num+1), num))  
                       
           self.typesAndBytes[type] = num

        #add this to our dictionary
        else:
           self.typesAndBytes[type] = num

    def handle(self):
            global COORDINATOR_IP
            global LR_IP
            global FOLLOWER_IP
            global SENSOR_IP
            

            last_message_number = 0

            inter_count = 0
            prev_count = 0
            prev_time = 0
            inter_time = 0

            goodMsgTotal = 0
            badMsgTotal = 0
            msgTotal = 0
            
            message = ''
            cur_thread = threading.current_thread()
            print("we have a connection from {0}".format(self.request.getpeername()))

            if SENSOR_IP in self.request.getpeername():
               print("The sensor has connected.")

            while True:
                data = self.request.recv(1) #96

                if data == MESSAGE_START_BYTE:

                   #Account for messages with count byte = '~'
                   if len(message) == 2:
                      type = message[1] #hex(ord(message[1]))
                      
                      if self.typesAndBytes.has_key(type):
                         if self.typesAndBytes[type] == 125:
                            message = message + data
                            continue

                   message = MESSAGE_START_BYTE




                elif data == MESSAGE_STOP_BYTE:

                   message = message + data

                   type = message[1] #hex(ord(message[1]))

                   #Account for messages with count byte = MESSAGE_STOP_BYTE
                   if len(message) == 3:
                      if self.typesAndBytes.has_key(type):
                         if self.typesAndBytes[type] == 45:
                            continue
                   
                   #inter_count = total_count - prev_count
                   #prev_count = total_count

                   #Print out message receive data
                   if type == MESSAGE_RCV_STAT:
                      #print >>sys.stderr, " ".join(hex(ord(n)) for n in message)
                      #print(inter_count - ord(message[8]))

                      #Test accuracy of receive timer callback, calculate receive rate
                      inter_time = time.time() - prev_time
                      prev_time = time.time()

                      if inter_time != 0 and len(message) > 8:
                         goodMsg = ord(message[8])
                         badMsg = ord(message[6])
                         goodMsgTotal = goodMsgTotal + goodMsg
                         badMsgTotal = badMsgTotal + badMsg
                         msgTotal = msgTotal + badMsg + goodMsg

                         print(">>> Messages received: {0}           Errors: Reported- {1}".format(goodMsg, badMsg))
                         print("    Received/sec: {0}    # Seconds: {1}".format((goodMsg+badMsg) / inter_time, inter_time))

                      message = ''
                      continue

                   if SENSOR_IP in self.request.getpeername():
                      SENSORprocessMsg(message)
                      
                   if len(message) > 2:
                      num = ord(message[2])

                   self.trackIncoming(type, int(num))

                   message = ''



                else:
                   message = message + data



            print("closing the connection")
            
 
  

class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    socketserver.ThreadingMixIn.daemon_threads = True
    pass

def client(ip, port, message):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    try:
        sock.sendall(bytes(message, 'ascii'))
        response = str(sock.recv(1024), 'ascii')
        print("Received: {}".format(response))
    finally:
        sock.close()


def signal_handler(signal, frame):
    print('Closing Server...')


if __name__ == "__main__":
    global server
    # Port 0 means to select an arbitrary unused port
    HOST, PORT = "192.168.42.1", 10000

    server = ThreadedTCPServer((HOST, PORT), ThreadedTCPRequestHandler)
    ip, port = server.server_address

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    server_thread = threading.Thread(target=server.serve_forever)
    # Exit the server thread when the main thread terminates
    server_thread.daemon = True
    server_thread.start()
    print("Server loop running in thread:", server_thread.name)

    #client(ip, port, "Hello World 1")

    signal.signal(signal.SIGINT, signal_handler)

    #pause until we get a control interrupt
    signal.pause()  
 
    server.shutdown()
    server.server_close()
    print("Server Closed.")

    #sys.exit(0)



