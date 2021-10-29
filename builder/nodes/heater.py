#!/usr/bin/env python


import rospy
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse
import RPi.GPIO as GPIO

"""
This is a node controling the Heating Device 
on/off a relay using GPIO directly from rasp PI 

SERVICE --> heatingMODE (input Boolean)

PIN HIGH --> HEATING DEVICE OFF
PIN LOW  --> HEATING DEVICE ON

"""

PIN_ID = 17

class HEAT_NODE():

    """ INIT    """
    def __init__(self):
        rospy.init_node('heating_node',anonymous=True,log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)
        rospy.Service('/heating_node/heatingMode', SetBool, self.srv_heatingMODE)
        self.heatingMODE = False
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(PIN_ID,GPIO.OUT)
        GPIO.output(17,GPIO.HIGH)

        rospy.loginfo("Initiating Heat NODE")
        rospy.spin()


    """
    Control the GPIO on/off
    """  
    def cntrl_gpio(self):
        rospy.logdebug(f"HEATER --> activate Heating : {self.heatingMODE}")
        if(self.heatingMODE):
            # open pin
            GPIO.output(PIN_ID,GPIO.HIGH)
        else:
            # close pin
            GPIO.output(PIN_ID,GPIO.LOW)
    

    """
    service for controling the heating device
    """
    def srv_heatingMODE(self,SetBoolRequest):
        self.heatingMODE = SetBoolRequest.data
        self.cntrl_gpio()
        if(SetBoolRequest.data):
            return SetBoolResponse(success = True,message = "Heating mode ON")
        return SetBoolResponse(success = True,message = "Heating mode OFF")

    def shutdown(self):
        self.heatingMODE = False
        self.cntrl_gpio()
        GPIO.cleanup()
        rospy.logerr("HEATER --> SHUT DOWN")




"""  MAIN    """
if __name__ == '__main__':
    try:
        heat_node = HEAT_NODE()
    except rospy.ROSInterruptException:
        rospy.logerr("HEATER --> I THINK NODE DIED...")

