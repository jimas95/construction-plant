#!/usr/bin/env python


import rospy
from rospy.core import logerr
from std_srvs.srv import SetBool
from std_srvs.srv import Empty
import actionlib
import builder.msg
import time

"""
THIS NODE WILL DRAW/PRINT A SMILEY FACE
This is a node controling the path planer
Set coolling or printing mode, and where to go print and when to go for refill
"""


FLOW = [
    ("IDLE" ,"NULL"),
    ("GOTO" ,["PREPRINT",True]),
    ("GOTO" ,["REFILL"  ,False]),
    ("FILL","NULL"),
    ("GOTO",["PREPRINT",True]),
    ("GOTO",["EYE_1",True]),
    ("HEAT",True),
    ("IDLE","NULL"),
    ("HEAT",False),
    ("IDLE","NULL"),
    ("GOTO",["PREPRINT",False]),
    ("GOTO",["EYE_2",True]),
    ("HEAT",True),
    ("IDLE","NULL"),
    ("HEAT",False),
    ("GOTO",["PREPRINT",False]),
    ("GOTO",["CIRCLE",True]),
    ("END","NULL")]


GOTO_POS = {
"REFILL":   builder.msg.PathPlanInfoGoal(centerX = 0.25, centerY =  0.0   , reverse=True ,range = 0.0  , init_time = 0   , step_size = 1.0, mode = "POINT", repeat = 0),
"PREPRINT": builder.msg.PathPlanInfoGoal(centerX = 0.5 , centerY =  0.0   , reverse=True ,range = 0.0  , init_time = 0   , step_size = 1.0, mode = "POINT", repeat = 0),
"CIRCLE":   builder.msg.PathPlanInfoGoal(centerX = 1.35, centerY =  0.3   , reverse=True ,range = 0.4  , init_time = 0   , step_size = 20 , mode = "CIRCLE",repeat = 0),
"EYE_1":    builder.msg.PathPlanInfoGoal(centerX = 1.5 , centerY =  0.15  , reverse=True ,range = 0.0  , init_time = 0   , step_size = 1.0, mode = "POINT", repeat = 0),
"EYE_2":    builder.msg.PathPlanInfoGoal(centerX = 1.2 , centerY =  0.15  , reverse=True ,range = 0.0  , init_time = 0   , step_size = 1.0, mode = "POINT", repeat = 0)
}

class BRAIN():

    """ INIT    """
    def __init__(self):
        rospy.init_node('brain',anonymous=True,log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initiating smiley face NODE")

        self.use_real = rospy.get_param("/use_real")

        self.current_state = "START"
        self.command = False

        self.STATES = {
            "START":self.start,
            "IDLE":self.idle,
            "HEAT":self.heat,
            "GOTO":self.goto,
            "FILL":self.refill,
            "END":rospy.signal_shutdown
            }

        # Creates the SimpleActionClient and wait for server to come up
        self.planner_action = actionlib.SimpleActionClient('action_planner', builder.msg.PathPlanInfoAction)
        self.planner_action.wait_for_server()

    def start(self,null):
        rospy.loginfo("INITILIZING 3D PRINTING A SMILEY FACE")
        
    def idle(self,null):
        response = 'n'
        while(response!='y'):
            response = input("STOP IDLE MODE ? ")
        
        
    def heat(self,mode):
        if(mode):
            rospy.loginfo("STATE --> SET HEATING PLATE ON")
            self.heater_mode(mode)
        else:
            rospy.loginfo("STATE --> SET HEATING PLATE OFF")
            self.heater_mode(mode)
        
    def goto(self,input):
        gowhere = input[0]
        plan = GOTO_POS[gowhere]
        plan.reverse = input[1] 
        rospy.loginfo(f"STATE --> GOTO --> {gowhere}")
        rospy.loginfo(f"STATE --> GOTO --> START ACTION")
        self.planner_action.send_goal(plan) # Sends the goal to the action server.
        self.planner_action.wait_for_result()
        # while(not self.planner_action.wait_for_result(timeout=rospy.Duration(5))):
            # rospy.loginfo(f"STATE --> GOTO --> WAIT FOR ACTION")


    def refill(self,null):
        rospy.loginfo("STATE --> CALLING REFILL SERVICE")
        self.refillBuilder()


    def execute_state(self):
        rospy.loginfo(f"STATE --> {self.current_state} --> START")
        time.sleep(0.1)
        self.STATES[self.current_state](self.command)
        rospy.loginfo(f"STATE --> {self.current_state} --> DONE")
        time.sleep(0.1)
        rospy.loginfo("-----------------")
        rospy.loginfo("")
        time.sleep(0.1)
    
    def print_next_state(self):
        if(len(FLOW)>0):
            rospy.loginfo(f"NEXT STATE IS --> {FLOW[0]}")
        else:
            rospy.logwarn("something is wrong, we run out of stages")

    def next_state(self):
        if(len(FLOW)>0):
            (self.current_state,self.command) = FLOW.pop(0)
        else:
            rospy.logwarn("something is wrong, we run out of stages")


    def shutdown(self):
        rospy.logerr("BRAIN --> SHUT DOWN")


    ############################  SERVICES ############################
    """
    set planner on/off
    """
    def stopPlan(self,mode):
        rospy.wait_for_service('/path_plan/stop')
        try:
            call_srv = rospy.ServiceProxy('/path_plan/stop', SetBool)
            resp1 = call_srv(mode)
            rospy.loginfo("BRAIN --> request planer to stop " + resp1.message)
        except rospy.ServiceException as e:
            rospy.logerr("BRAIN --> Service call failed: %s"%e)


    """
    call refilling of candles, ACTIVATE ARM
    """
    def refillBuilder(self):
        rospy.wait_for_service("/px100/refill")
        try:
            call_srv = rospy.ServiceProxy("/px100/refill", Empty)
            resp1 = call_srv()
            rospy.loginfo("BRAIN  --> ACTIVATE ARM ")
            rospy.loginfo("BRAIN  --> REFILLING BUILDER WITH CANDLES ")
        except rospy.ServiceException as e:
            rospy.logerr("BRAIN  --> Service call failed: %s"%e)


    """
    ACTIVATE/DEACTIVATE HEATING PLATE
    """
    def heater_mode(self,mode):
        if(not self.use_real): return
        rospy.wait_for_service("/heating_node/heatingMode")
        try:
            call_srv = rospy.ServiceProxy("/heating_node/heatingMode", SetBool)
            resp1 = call_srv(mode)
        except rospy.ServiceException as e:
            rospy.logerr("BRAIN  --> Service call failed: %s"%e)


""" INIT smiley face   """
def start():
    brain_node = BRAIN()
    rate = rospy.Rate(1) # publish freacuancy 
    rospy.loginfo("okei lets do it")
    
    # main loop
    while not rospy.is_shutdown():
        brain_node.execute_state()
        brain_node.print_next_state()
        brain_node.next_state()
        rate.sleep()
            
"""  MAIN    """
if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        rospy.logerr("BRAIN --> I THINK NODE DIED...?")