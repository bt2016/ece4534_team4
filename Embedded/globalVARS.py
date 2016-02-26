"""
Define any variables below that are global to the entire system.

This file is inherited by many other files
"""

#COORDINATOR_IP = "192.168.42.51"
#LR_IP = "192.168.42.52"
#FOLLOWER_IP = "192.168.42.53"
#SENSOR_IP = "192.168.42.54"

COORDINATOR_IP = "192.168.42.51"
LR_IP = "192.168.42.54"
FOLLOWER_IP = "192.168.42.53"
SENSOR_IP = "192.168.42.52"
BROOKELAPTOP_IP = "192.168.42.11"

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

#Message types diverted to coordinator
TYPE_SENSORARRAY = chr(233)
TYPE_RECEIVE_STAT = chr(254) #0xfe - Received from all PICs
TYPE_LR_SENSOR = chr(247)
TYPE_LR_ENCODER = chr(127)

#Follower rover message types, may not be needed by Pi
TYPE_FR_ENCODER = chr(111)
TYPE_FR_US = chr(95)
TYPE_FR_IR = chr(159)

#Message types from the sensor for MS3
TYPE_BROOKE_APPENDPOLAR = chr(97)
TYPE_BROOKE_DISPLAY = chr(100)
TYPE_BROOKE_CLEAR = chr(99)





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
