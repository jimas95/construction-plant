#!/usr/bin/env python


import re
import rospy
from brain.srv import myString,myStringRequest,myStringResponse


class TIME_MANAGMENT():

    """ INIT    """
    def __init__(self,cycle):
        rospy.init_node('time_managment',anonymous=False,log_level=rospy.INFO)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initiating RAMP NODE")
        self.time = 0 
        self.print_time = 4*60
        self.cool_time = 2*60

        
        self.cycle = cycle
        self.id = -1
        self.state = ["IDLE",15]
        self.line_choose = False

    def update(self):
        self.time = self.time + 1 # update time
        rospy.loginfo(f"TIME_MANAGMENT --> STATE {self.state[0]}")
        rospy.loginfo(f"TIME_MANAGMENT --> {self.time} > {self.state[1]}")

        if(self.time > self.state[1]):
            # lets play a new move
            self.id = self.id + 1
            if(self.id == len(self.cycle)):
                return True
            else:
                self.state = self.cycle[self.id]
            self.call_srv()
            self.time = 0 
        return False



    """
    call service to set mode states on ramp builder
    """
    def  call_srv(self):
        rospy.loginfo(f"TIME_MANAGMENT --> call RAMP service --> {self.state[0]}")
        
        # call service
        rospy.wait_for_service("/brain_mode")
        try:
            msg = myStringRequest()
            msg.mode = self.state[0]
            call_srv = rospy.ServiceProxy("/brain_mode", myString)
            resp1 = call_srv(msg)
            rospy.loginfo(f"TIME_MANAGMENT  --> SET brain_mode to {self.state[1]} ")
        except rospy.ServiceException as e:
            rospy.logerr("TIME_MANAGMENT  --> Service call failed: %s"%e)

    """
    Shut down node
    """
    def shutdown(self):
        rospy.logerr("TIME_MANAGMENT --> SHUT DOWN")

    def set_mode(self,cycle):
        self.cycle = cycle
        self.id = 0 
        self.time = 0

""" INIT smiley face   """
def start():
    cycle_1 = (("idle",2),("refill",28),("l2",5.2*60),("next",40),("refill",25))
    cycle_2 = (("idle",2),("refill",28),("l1",5.2*60),("next",40),("refill",25))

    cycle_1 = (("idle",2),("l2",5.2*60),("next",30))
    cycle_2 = (("idle",2),("l1",5.2*60),("next",30))

    # cycle__ = (("idle",2),("l2",15*60),("next",1*60),("next",28),("refill",15))
    cycle_selection = True
    time_managment = TIME_MANAGMENT(cycle_1)
    rate = rospy.Rate(1) # publish freacuancy 
    rospy.loginfo("okei lets do it")
    


    time_managment.timer = 0 
    # main loop
    while not rospy.is_shutdown():
        
        result = time_managment.update()
        if(result):
            if(cycle_selection):
                time_managment.set_mode(cycle_2)
                # time_managment.set_mode(cycle__)
            else:
                time_managment.set_mode(cycle_1)
                # time_managment.set_mode(cycle__)
            cycle_selection = not cycle_selection

        rate.sleep()
            
"""  MAIN    """
if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        rospy.logerr("BRAIN --> I THINK NODE DIED...?")