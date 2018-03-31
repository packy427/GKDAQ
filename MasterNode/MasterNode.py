"""@package MasterNode"""
# -----------------------------------------------------------------------------
#   Filename   : MasterNode.py
#   Title      : Master Node Main
#   Author     : Patrick Kennedy (PK3)
#   Created    : 02/28/2018
#   Modified   : 02/28/2018
#   Version    : 0.1
#   Description:
#
# -----------------------------------------------------------------------------
#!/usr/bin/python

import time
import Adafruit_CharLCD as CharLCD
import can
from enum import Enum
import logging
logging.basicConfig(level=logging.INFO)

# == GLOBAL MEASUREMENT VARIABLES == #
M_ENG_TEMP = 0
M_EXH_TEMP = 0
M_ENG_SPEED = 0
M_AXL_SPEED = 0
M_THR_POS = 0
M_BRK_POS = 0
M_STR_ANGLE = 0
M_AMB_TEMP = 0
M_ACC_X = 0
M_ACC_Y = 0
M_ACC_Z = 0
M_GYR_X = 0
M_GYR_Y = 0
M_GYR_Z = 0
M_HEADING = 0
M_GPS_LAT = 0
M_GPS_LONG = 0
M_HUMIDITY = 0
M_TEST_POT = 0


# == FUNCTIONS == #
def decode_message(can_msg):
    # global M_TEST_POT uncomment this is calling just M_TEST_POT doesn't work
    # Test Potentiometer
    if can_msg.abitration_id == CANID.TESTPOT:
        M_TEST_POT = from_byte_array(can_msg.data, can_msg.dlc)
        print("<i> Test Pot: " + M_TEST_POT)
    # Engine head temperature
    elif can_msg.abitration_id == CANID.ENGINETEMP:
        print("<i> Received ")
    # Exhaust gas temperature
    elif can_msg.abitration_id == CANID.EXHAUSTTEMP:
        print("<i> Received ")
    # Engine speed
    elif can_msg.abitration_id == CANID.ENGINESPEED:
        print("<i> Received ")
    # GPS
    elif can_msg.abitration_id == CANID.GPS:
        print("<i> Received ")
    # Acceleration
    elif can_msg.abitration_id == CANID.ACCELERATION:
        print("<i> Received ")
    # Heading
    elif can_msg.abitration_id == CANID.HEADING:
        print("<i> Received ")
    # Axle speed
    elif can_msg.abitration_id == CANID.AXLESPEED:
        print("<i> Received ")
    # Gyration
    elif can_msg.abitration_id == CANID.GYRATION:
        print("<i> Received ")
    # Throttle position
    elif can_msg.abitration_id == CANID.THROTTLEPOSITION:
        print("<i> Received ")
    # Brake position
    elif can_msg.abitration_id == CANID.BRAKEPOSITION:
        print("<i> Received ")
    # Steering angle
    elif can_msg.abitration_id == CANID.STEERINGANGLE:
        print("<i> Received ")
    # Ambient temperature
    elif can_msg.abitration_id == CANID.AMBIENTTEMP:
        print("<i> Received ")
    # Humidity
    elif can_msg.abitration_id == CANID.HUMIDITY:
        print("<i> Received ")
    else:
        print("<x> No matching CAN ID")


def from_byte_array(array, bytes):
    data = 0
    for i in range(0, bytes):
        data += (array[i] << (8 * i))
    return data


# == CAN IDS == #
class CANID(Enum):
    CAL_START        = 0x01
    CAL_CHANGEADDR   = 0x02
    CAL_CHANGEIO     = 0x03
    CAL_RESET        = 0x04
    CAL_EXIT         = 0x0F
    ENGINETEMP       = 0x10
    EXHAUSTTEMP      = 0x11
    ENGINESPEED      = 0x12
    GPS              = 0x20
    ACCELERATION     = 0x21
    HEADING          = 0x22
    AXLESPEED        = 0x23
    GYRATION         = 0x24
    THROTTLEPOSITION = 0x30
    BRAKEPOSITION    = 0x31
    STEERINGANGLE    = 0x32
    AMBIENTTEMP      = 0x40
    HUMIDITY         = 0x41
    TESTPOT          = 0x7F


# == LCD INIT == #
LCD_RS = 17
LCD_EN = 27
LCD_D4 = 25
LCD_D5 = 24
LCD_D6 = 23
LCD_D7 = 18

LCD_COLS = 20
LCD_ROWS = 4

LCD = CharLCD.Adafruit_CharLCD(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7,
                               LCD_COLS, LCD_ROWS)
LCD.clear()
LCD.message("<i> LCD Init")
print("<i> LCD initialized")
time.sleep(0.5)

# == CAN BUS INIT == #
can_interface = 'can0'
can_bustype = 'socketcan_native'
bus = can.interface.Bus(can_interface, bustype=can_bustype)

LCD.set_cursor(0, 1)
LCD.message("<i> CAN Init")
print("<i> CAN initialized")

LCD.set_cursor(0, 2)
LCD.message("Test Pot: ")

# == MAIN LOOP == #
def main():
	while 1:
	    rx_message = bus.recv(0.5)     # Read receive buffer, timeout after 0.5 secs
	    if rx_message is not None:
        	print("<i> Receive buffer has message")
       		decode_message(rx_message)
        	LCD.set_cursor(12, 2)
        	LCD.message(M_TEST_POT)
   	else:
        	print("<i> Receive buffer empty")

if __name__ == "__main__":
	main()






