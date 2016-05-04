"""
Define any variables below that are global to the entire system.

This is an example of a message in our message format:

example_msg = '~12dddddd.'
where 1 is the type, and 2 is the msg count, and d's are data

This file is inherited by many other files
"""

#This value is communicated to the coordinator upon connection
NUMBER_OF_TOKENS = 5

#COORDINATOR_IP = "192.168.42.51"
#LR_IP = "192.168.42.52"
#FOLLOWER_IP = "192.168.42.53"
#SENSOR_IP = "192.168.42.54"

COORDINATOR_IP = "192.168.42.51"
#LR_IP = "192.168.42.52"
FOLLOWER_IP = "192.168.42.53"
LR_IP = "192.168.42.52"

#SENSOR_IP = "192.168.42.52"
#BROOKELAPTOP_IP = "192.168.42.54"
SENSOR_IP = "192.168.42.54"
BROOKELAPTOP_IP = "192.168.42.64"

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

#Message types diverted to coordinator
TYPE_SENSORARRAY = chr(233)
TYPE_RECEIVE_STAT = chr(254) #0xfe - Received from all PICs
TYPE_LR_SENSOR = chr(247)
TYPE_LR_ENCODER = chr(127)

#Follower rover message types, may not be needed by Pi
TYPE_FR_ENCODER = chr(111)
TYPE_FR_DIST = chr(95)
TYPE_FR_IR = chr(159)

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
TYPE_SENSOR_ACK = chr(113)

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


