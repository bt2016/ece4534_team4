from keyPoll import *
from SENSOR_PROC import processMsgFromDaniel as process
from globalVARS import *

with KeyPoller() as keyPoller:
    print("HERE")
    while True:
        keyIn = keyPoller.poll()
        if not keyIn is None:
           if keyIn == "g":
              process(TYPEC_GO, 0)
           elif keyIn == "s":
              process(TYPEC_STOP, 0)
           elif keyIn == "t":
              process(TYPEC_LR_SENSOR_TO_FR, 0)
           elif keyIn == "r":
              process(TYPE_HISTORY, 0)
           elif keyIn == "z":
              process(TYPE_FR_ACK, 0)
           elif keyIn == "0":
              process(TYPE_MS4SET, 0)
           elif keyIn == "1":
              process(TYPE_MS4SET, 1)
           elif keyIn == "2":
              process(TYPE_MS4SET, 2)
           elif keyIn == "3":
              process(TYPE_MS4SET, 3)
           elif keyIn == "4":
              process(TYPE_MS4SET, 4)
           elif keyIn == "5":
              process(TYPE_MS4SET, 5)
           elif keyIn == "6":
              process(TYPE_MS4SET, 6)
           elif keyIn == "7":
              process(TYPE_MS4SET, 7)