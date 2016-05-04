"""
Define any variables below that are global to the entire system.

This is an example of a message in our message format:

example_msg = '~12dddddd.'
where 1 is the type, and 2 is the msg count, and d's are data

This file is inherited by many other files
"""

#This value is communicated to the coordinator upon connection
NUMBER_OF_TOKENS = 3

#COORDINATOR_IP = "192.168.42.51"
#LR_IP = "192.168.42.52"
#FOLLOWER_IP = "192.168.42.53"
#SENSOR_IP = "192.168.42.54"

#COORDINATOR_IP = "192.168.42.54" #THIS SHOULD BE .51, changed by John on 4/25 for testing with Daniel
COORDINATOR_IP = "192.168.42.51" #THIS SHOULD BE .51, changed by John on 4/25 for testing with Daniel
#LR_IP = "192.168.42.52"
FOLLOWER_IP = "192.168.42.53"
LR_IP = "192.168.42.52"

#SENSOR_IP = "192.168.42.52"
#BROOKELAPTOP_IP = "192.168.42.54"
#SENSOR_IP = "192.168.42.51" #THIS SHOULD BE .54, changed by John on 4/25 for testing with Daniel
SENSOR_IP = "192.168.42.54" #THIS SHOULD BE .54, changed by John on 4/25 for testing with Daniel
BROOKELAPTOP_IP = "192.168.42.64"

DANIEL_IP = "192.168.42.100"


#these are the LISTENING ports
COORDINATOR_PORT = 10000
LR_PORT = 10000
FOLLOWER_PORT = 10000
SENSOR_PORT = 10000

MESSAGE_START_BYTE = '~'
MESSAGE_STOP_BYTE = '.'

#Coordinator sent message types
TYPEC_LR_SENSOR_TO_FR = chr(143)
TYPEC_MOTOR_CTRL = chr(94)
TYPEC_LR_HANDSHAKE = chr(79)
TYPEC_TKN_REQUEST = chr(93)
TYPEC_ACK_TOKEN = chr(80)
TYPEC_CLEAR_MAP = chr(81)
TYPEC_UPDATE_MAP = chr(82)
TYPEC_MAP_DATA = chr(83)
TYPEC_MAKE_MOVE = chr(84)
TYPEC_END_MAP_DATA = chr(85)
TYPEC_TOKEN_FOUND = chr(86)
TYPEC_LR_MOVE_COMPLETE = chr(87)

TYPEC_TOKEN_FOUND_ACK = chr(96)

TYPEC_MOVE_FORWARD = chr(48) 
TYPEC_TURN_LEFT = chr(49)
TYPEC_TURN_RIGHT = chr(50)
TYPEC_TURN_UP = chr(52)
TYPEC_TURN_DOWN = chr(53)
TYPEC_ROTATE = chr(54)

TYPE_FR_FOUND_TOKEN = chr(241)

TYPEC_ACK_SENSOR_DATA = chr(113)


TYPEC_CONTINUE = chr(88)
TYPEC_SKIP = chr(89)


TYPEC_GO = chr(153) # 0x99
TYPEC_STOP = chr(145) # 0x91

#Message types diverted to coordinator
TYPE_SENSORARRAY = chr(233)
TYPE_RECEIVE_STAT = chr(254) #0xfe - Received from all PICs
TYPE_LR_SENSOR = chr(247)
TYPE_LR_ENCODER = chr(127)

#Follower rover message types, may not be needed by Pi
TYPE_FR_ENCODER = chr(111)
TYPE_FR_DIST = chr(95)
TYPE_FR_IR = chr(159)
TYPE_PID = chr(68)
TYPE_DISTPID = chr(165)
TYPE_FR_ACK = chr(205)
TYPE_FR_TARGETDATA = chr(171)
TYPE_FR_FOUNDTOKEN = chr(241)
TYPE_ADC = chr(24)
TYPE_HISTORY = chr(101)
TYPE_FR_STATUS = chr(102)
TYPE_MS4SET = chr(82)



#Message types from the sensor for MS3
TYPE_BROOKE_APPENDPOLAR = chr(97)
TYPE_BROOKE_DISPLAY = chr(100)
TYPE_BROOKE_CLEAR = chr(99)
TYPE_BROOKE_ECHO = chr(101)

#Message types from the sensor for MS4
TYPE_SENSOR_APPENDMAP = chr(109)
TYPE_SENSOR_APPENDLINES = chr(108)
TYPE_SENSOR_APPENDTARGETS = chr(116)
TYPE_SENSOR_CLEARMAP = chr(77)
TYPE_SENSOR_CLEARLINES = chr(76)
TYPE_SENSOR_CLEARTARGETS = chr(84)
TYPE_SENSOR_CLEARALL = chr(67)
TYPE_SENSOR_DISPLAYFULLMAP = chr(100)
TYPE_SENSOR_DISPLAYFIELD = chr(68)
TYPE_SENSOR_ECHO = chr(101)

#The coordinator should send this message type to the sensor 
#to request a new list of obstacles
#NOTE: the message must have 10 characters or else the pic won't recognize it
TYPE_SENSOR_UPDATEREQUESTED = chr(117)
TYPE_SENSOR_SINGLEREQUESTED = chr(118)
TYPE_SENSOR_MULTIPLEREQUESTED = chr(119)

#The coordinator will receive this message type when the sensor
#has finished sending the list of obstacles
TYPE_SENSOR_ENDUPDATE = chr(90)



MOTOR_FORWARD = chr(17)
MOTOR_BACKWARD = chr(85)
MOTOR_STOP = chr(4)



#Expected UART message rates per second
LR_SENSOR_TOKEN = 2        # 2 reports per second for demonstration
LR_ENCODER_DATA = 10       # 100ms per message

CD_MOTOR_CONTROL = 10      # 100ms per message
CD_SENSOR_RESPONSE = 4     # 2 for each token found message

FR_SENT_DATA = 0           # Sends nothing

SENSOR_PROCESSED_DATA = 20      # Message every 50ms

EXPECTED_LR_SEND = LR_SENSOR_TOKEN + LR_ENCODER_DATA           
EXPECTED_FR_SEND = 0
EXPECTED_SA_SEND = SENSOR_PROCESSED_DATA
EXPECTED_CD_SEND = CD_MOTOR_CONTROL + (LR_SENSOR_TOKEN * 2)

EXPECTED_LR_RECEIVE = CD_MOTOR_CONTROL + LR_SENSOR_TOKEN
EXPECTED_FR_RECEIVE = LR_SENSOR_TOKEN
EXPECTED_SA_RECEIVE = 0
EXPECTED_CD_RECEIVE = SENSOR_PROCESSED_DATA + LR_SENSOR_TOKEN + LR_ENCODER_DATA

#Coordinator memory variables
coordinator_token_number_sent = False
coordinator_ack_token_number = False

#Simulation variables:
ERROR_AMT = 2 #Max amount of error for moves greater than 2 cm's 
ROVER_WIDTH = 7 #primarily used to determine distance to tokens


