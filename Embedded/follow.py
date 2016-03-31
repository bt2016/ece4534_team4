import socket
import threading
import SocketServer as socketserver

#for controlled close:
import sys
import signal
import time

#import all our variables
from globalVARS import *

from SENSOR_PROC import processMsgToCoordinator as sendToCoordinator
from SENSOR_PROC import processMsgFromCoordinator as processCoordinatorMsg
from SENSOR_PROC import processMsgFromBrooke as processBrookeMsg

class ThreadedTCPRequestHandler(socketserver.BaseRequestHandler):

    typesAndBytes = {}
    #total_count = 0

    def motorData(self, rightDir, rightSpd, leftDir, leftSpd):

       right = rightSpd;
       left = leftSpd;

       # Determine minimum value
       if leftSpd >= rightSpd:
          min = rightSpd
       else:
          min = leftSpd

       #If moving forward on both
       if leftDir == MOTOR_FORWARD and rightDir == MOTOR_FORWARD:

          if leftSpd == rightSpd:
             dir = "STRAIGHT AHEAD     "
          elif leftSpd > rightSpd:
             dir = "FORWARD, SKEW RIGHT"
          else:
             dir = "FORWARD, SKEW LEFT "

       elif leftDir == MOTOR_FORWARD and rightDir == MOTOR_BACKWARD:
          right = -right;
          dir = "ROTATE RIGHT       "
       elif leftDir == MOTOR_BACKWARD and rightDir == MOTOR_FORWARD:
          left = -left;
          dir = "ROTATE LEFT        "
       elif leftDir == MOTOR_BACKWARD and rightDir == MOTOR_BACKWARD:
          right = -right;
          left = -left;
          dir = "TOTAL REVERSE      "
       elif leftDir == MOTOR_STOP and rightDir == MOTOR_STOP:
          dir = "ROVER STOPPED      "  

       if min == 0:
          ect = "       "
       elif min < 40:
          ect = "(Slow) "
       elif min < 70:
          ect = "(Med)  "
       else:
          ect = "(Fast) "

       ion = " --- Right: " + str(right) + " / Left: " + str(left)

       return dir + ect + ion

    def printIRDir(self, front, rear, left, right):

       if front > 40:
          fmes = "HIT"
       else:
          fmes = "---"

       if rear > 40:
          rmes = "HIT"
       else:
          rmes = "---"

       if left > 40:
          lmes = "HIT"
       else:
          lmes = "---"

       if right > 40:
          gmes = "HIT"
       else:
          gmes = "---"

       print("<IR RX>  Front: {0}  /  Rear: {1}  /  Left: {2}  /  Right: {3}\n".format(fmes, rmes, lmes, gmes))


    def trackIncoming(self, type, num):
  
        #its in our dictionary, check it
        if self.typesAndBytes.has_key(type):
          
           last_msg_num = self.typesAndBytes[type]

           if last_msg_num == 255: 
              last_msg_num = -1

           #if (last_msg_num + 1) != num:
           #   print("Type {0} skipped message. I expected #{1} but received #{2}".format(hex(ord(type)), (last_msg_num+1), num))  
                       
           self.typesAndBytes[type] = num

        #add this to our dictionary
        else:
           self.typesAndBytes[type] = num

    def handle(self):
            global COORDINATOR_IP
            global LR_IP
            global FOLLOWER_IP
            global SENSOR_IP
            global BROOKELAPTOP_IP
            

            last_message_number = 0
            sent_messages = 0

            inter_count = 0
            prev_count = 0
            prev_time = 0
            inter_time = 0

            goodMsgTotal = 0
            badMsgTotal = 0
            msgTotal = 0

            minuteCount = 0
            minuteTick = 0

            prevSensorData = 0

            message = ''
            cur_thread = threading.current_thread()
            print("we have a connection from {0}".format(self.request.getpeername()))

            #IDENTIFY YOURSELF!
            name = "DEFAULT"
            if SENSOR_IP in self.request.getpeername():
               print("The sensor has connected.")
               name = "SENSOR"
               txrate = EXPECTED_SA_SEND
               rxrate = EXPECTED_SA_RECEIVE
            elif COORDINATOR_IP in self.request.getpeername():
               print("The coordinator has connected.")
               name = "COORD"
               txrate = EXPECTED_CD_SEND
               rxrate = EXPECTED_CD_RECEIVE
            elif LR_IP in self.request.getpeername():
               print("The lead rover has connected.")
               name = "LEAD"
               txrate = EXPECTED_LR_SEND
               rxrate = EXPECTED_LR_RECEIVE
            elif FOLLOWER_IP in self.request.getpeername():
               print("The follower rover has connected.")
               name = "FOLLOW"
               txrate = EXPECTED_FR_SEND
               rxrate = EXPECTED_FR_RECEIVE
                

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

                   if len(message) == 0:
                      continue

                   message = message + data

                   type = message[1] #hex(ord(message[1]))

                   #Account for messages with count byte = MESSAGE_STOP_BYTE
                   if len(message) == 3:
                      if self.typesAndBytes.has_key(type):
                         if self.typesAndBytes[type] == 45:
                            continue
                   
                   #inter_count = total_count - prev_count
                   #prev_count = total_count

                  
                   if COORDINATOR_IP in self.request.getpeername():
                      processCoordinatorMsg(type, message)
                   else:
                      #sendToCoordinator(message)
                      if len(message) < 9:
                         continue
                      elif message[1] == TYPE_FR_DIST:
                         print("<SENSOR> reads {0:2} cm\n".format(ord(message[8])))
                      elif message[1] == TYPE_FR_IR and len(message) > 8:
                         self.printIRDir(ord(message[5]), ord(message[6]), ord(message[7]), ord(message[8]))
                         #print("<IR RX>  Front: {0:3}  Rear: {1:3}  Left: {2:3}  Right: {3:3}\n".format(ord(message[5]), ord(message[6]), ord(message[7]), ord(message[8])))
                      elif message[1] == TYPEC_MOTOR_CTRL and len(message) > 8:
                         motorString = self.motorData(message[5], ord(message[6]), message[7], ord(message[8]))

                         print("<MOTOR>  {0}\n".format(motorString))
                      #elif message[1] != TYPE_RECEIVE_STAT:
                      #   print(" ".join(hex(ord(n)) for n in message))
                            

                   sent_messages = sent_messages + 1
                      
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



