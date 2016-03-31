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

    def trackIncoming(self, type, num):
  
        #its in our dictionary, check it
        if self.typesAndBytes.has_key(type):
          
           last_msg_num = self.typesAndBytes[type]

           if last_msg_num == 255: 
              last_msg_num = -1

           if (last_msg_num + 1) != num:
              print("Type {0} skipped message. I expected #{1} but received #{2}".format(hex(ord(type)), (last_msg_num+1), num))  
                       
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
            elif BROOKELAPTOP_IP in self.request.getpeername():
                print("Brooke has connected.")
                


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

                   #Print out message receive data
                   if type == TYPE_RECEIVE_STAT:
                      #print >>sys.stderr, " ".join(hex(ord(n)) for n in message)
                      #print(inter_count - ord(message[8]))

                      #Test accuracy of receive timer callback, calculate receive rate
                      inter_time = time.time() - prev_time
                      prev_time = time.time()

                      if inter_time != 0 and len(message) > 8:
                         
                         if COORDINATOR_IP in self.request.getpeername():
                            minuteTick = minuteTick + 1
                            if minuteTick == 6:
                               minuteTick = 0
                               minuteCount = minuteCount + 1
                               print("Runtime: {0} minutes.\n".format(minuteCount))

                         goodMsg = ord(message[8]) + ord(message[7])*16
                         badMsg = ord(message[6]) + ord(message[5])*16
                         goodMsgTotal = goodMsgTotal + goodMsg
                         badMsgTotal = badMsgTotal + badMsg
                         msgTotal = msgTotal + badMsg + goodMsg

                         goodrate = goodMsg/inter_time
                         badrate = badMsg/inter_time
                         totalrate = (goodMsg+badMsg)/inter_time
                         pushrate = sent_messages/inter_time

                         #receiverate = abs((totalrate-rxrate)/(rxrate+0.000001))
                         if rxrate != 0:
                            receiverate = totalrate/rxrate

                         if totalrate == 0 and rxrate == 0:
                            rxreport = "RX Rate - GOOD!"
                         elif receiverate > 0.9 and receiverate < 1.1:
                            rxreport = "RX Rate - GOOD!"
                         elif receiverate > 1.1:
                            rxreport = "RX RATE FAST - {0:.3g}x SPEED".format(receiverate)
                         else:
                            rxreport = "RX RATE SLOW - {0:.3g}x SPEED".format(receiverate)


                         #sendcomp = abs((pushrate - txrate)/(txrate+0.000001))

                         if txrate != 0:
                           sendcomp = pushrate/txrate

                         if pushrate == 0 and txrate == 0:
                            txreport = "TX Rate - GOOD!"
                         elif sendcomp > 0.9 and sendcomp < 1.1:
                            txreport = "TX Rate - GOOD!"
                         elif sendcomp > 1.1:
                            txreport = "TX RATE FAST - {0:.3g}x SPEED".format(sendcomp)
                         else:
                            txreport = "TX RATE SLOW - {0:.3g}x SPEED".format(sendcomp)


                         if goodMsg == 0 and badMsg != 0:
                            errport = "BAD DATA"
                         elif goodMsg == 0:
                            errport = ""
                         elif badMsg == 0:
                            errport = "GOOD DATA"
                         elif (badMsg*10) < goodMsg:
                            errport = "MOSTLY GOOD DATA"
                         else:
                            errport = "ERRORS IN DATA"

                         if badMsg+goodMsg > 0 or sent_messages > 0:
                            print("<{0}> Messages received: {1} ({2:.3g}/s)  Sent: {3} ({4:.3g}/s)  Errors Reported: {5}".format(name, goodMsg, goodrate, sent_messages, pushrate, badMsg))
                            print("        {0} || {1} || {2}".format(rxreport, txreport, errport))
                            #print("        Rate/sec - Received: {0:.3g}     Sent: {1:.3g}".format(totalrate, pushrate))
                         else:
                            print("<{0}> received and sent 0 messages in {1:.3g}s. Expecting {2} received and {3] sent.".format(name, inter_time, txrate, rxrate))

                         print("")

                      message = ''
                      sent_messages = 0
                      continue



                   if COORDINATOR_IP in self.request.getpeername():
                      processCoordinatorMsg(type, message)


                   elif BROOKELAPTOP_IP in self.request.getpeername():
                      processBrookeMsg(type, message)
                   else:
                      sendToCoordinator(message)
                      if message[1] == TYPE_LR_SENSOR and message[3] == chr(88):
                         print("Sensor reads {0} cm".format(ord(message[8])))
                         #print(" ".join(hex(ord(n)) for n in MESSAGE))

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



