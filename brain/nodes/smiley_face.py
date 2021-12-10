#!/usr/bin/env python


import rospy
from rospy.core import NullHandler, logerr
from std_srvs.srv import SetBool
from std_srvs.srv import Empty
import actionlib
import builder.msg
import time
from math import pi
import copy

"""
THIS NODE WILL DRAW/PRINT A SMILEY FACE
This is a node controling the path planer
Set coolling or printing mode, and where to go print and when to go for refill
"""


FLOW = [
    # start
    ("IDLE" ,"NULL"),
    ("IDLE" ,"NULL"),
    ("GOTO" ,["PREPRINT",True]),

    # back and forth and refill
    ("GOTO" ,["PREFILL" ,False]),
    ("GOTO" ,["REFILL"  ,False]),
    ("FILL","NULL"),
    ("GOTO",["PREPRINT",True]),
   
    # print both eyes
    ("GOTO",["EYE_1",True]),
    ("HEAT",True),
    ("IDLE","NULL"),
    ("HEAT",False),
    ("IDLE","NULL"),
    ("GOTO",["EYE_2",False]),
    ("HEAT",True),
    ("IDLE","NULL"),
    ("HEAT",False),
    ("IDLE","NULL"),
    ("GOTO",["PREPRINT",False]),
    ("IDLE","NULL"),

    # print mouth
    ("GOTO" ,["PREFILL" ,False]),
    ("GOTO" ,["REFILL" ,False]),
    ("FILL","NULL"),
    ("GOTO",["PREPRINT",True]),
    ("GOTO",["CENMOUTH" ,True]),
    ("HEAT",True),
    ("GOTO",["MOUTH"   ,True]),
    ("INF",False),
    ("HEAT",False),
    ("GOTO",["MOUTH"   ,True]),
    ("INF",False),
    ("IDLE","NULL"),
    ("GOTO",["PREPRINT",False]),

    # print head
    ("GOTO" ,["PREFILL" ,False]),
    ("GOTO" ,["REFILL"  ,False]),
    ("FILL","NULL"),
    ("FILL","NULL"),
    ("GOTO",["PREPRINT",True]),
    ("GOTO",["SEMICIR" ,True]),
    ("HEAT",True),
    ("GOTO",["CIRCLE"  ,True]),
    ("INF",True),
    ("HEAT",False),
    ("GOTO",["CIRCLE"  ,True]),
    ("INF",False),
    ("GOTO",["PREPRINT",False]),
    ("END","NULL")]


GOTO_POS = {
"REFILL":   builder.msg.PathPlanInfoGoal(centerX = 0.402, centerY = -0.02  , reverse=True ,range = 0.0  , init_time = 1   , step_size = 1.0, mode = "POINT",  direction = 0  , printMD = 0),
"PREFILL":  builder.msg.PathPlanInfoGoal(centerX = 0.5  , centerY = -0.02  , reverse=True ,range = 0.0  , init_time = 1   , step_size = 1.0, mode = "POINT",  direction = 0  , printMD = 0),
"PREPRINT": builder.msg.PathPlanInfoGoal(centerX = 0.6  , centerY =  0.15  , reverse=True ,range = 0.0  , init_time = 1   , step_size = 1.0, mode = "POINT",  direction = 0  , printMD = 0),
"CIRCLE":   builder.msg.PathPlanInfoGoal(centerX = 1.45 , centerY =  0.3   , reverse=True ,range = 0.35 , init_time = 0   , step_size = 40 , mode = "CIRCLE", direction = 0  , printMD = 0),
"SEMICIR":  builder.msg.PathPlanInfoGoal(centerX = 1.45 , centerY =  0.3   , reverse=True ,range = 0.35 , init_time = pi  , step_size = 30 , mode = "CIRCLE", direction = 0  , printMD = 0),
"EYE_1":    builder.msg.PathPlanInfoGoal(centerX = 1.6  , centerY =  0.15  , reverse=True ,range = 0.0  , init_time = 1   , step_size = 1.0, mode = "POINT",  direction = 0  , printMD = 0),
"EYE_2":    builder.msg.PathPlanInfoGoal(centerX = 1.4  , centerY =  0.15  , reverse=True ,range = 0.0  , init_time = 1   , step_size = 1.0, mode = "POINT",  direction = 0  , printMD = 0),
"MOUTH":    builder.msg.PathPlanInfoGoal(centerX = 1.5  , centerY =  0.3   , reverse=True ,range = 0.15 , init_time = 0   , step_size = 20  , mode = "LINE",   direction = 0  , printMD = 0),
"CENMOUTH": builder.msg.PathPlanInfoGoal(centerX = 1.5  , centerY =  0.3  , reverse=True ,range = 0.0  , init_time = 1   , step_size = 1.0 , mode = "POINT",  direction = 0  , printMD = 0)
}

class BRAIN():

    """ INIT    """
    def __init__(self):
        rospy.init_node('brain',anonymous=False,log_level=rospy.INFO)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initiating smiley face NODE")

        self.use_real   = rospy.get_param("/use_real")
        self.debug_mode = rospy.get_param("/debug_mode",default=False)

        self.current_state = "START"
        self.command = False
        self.id = 0 

        self.STATES = {
            "START":self.start,
            "IDLE":self.idle,
            "HEAT":self.heat,
            "GOTO":self.goto,
            "FILL":self.refill,
            "SLEEP_ARM":self.set_sleep_arm,
            "INF" :self.repeat,
            "END":rospy.signal_shutdown
            }

        # Creates the SimpleActionClient and wait for server to come up
        self.planner_action = actionlib.SimpleActionClient('action_planner', builder.msg.PathPlanInfoAction)
        self.planner_action.wait_for_server()

    def start(self,null):
        rospy.loginfo("INITILIZING 3D PRINTING A SMILEY FACE")
        self.heatMODE = False
        self.heater_mode(False)

        
    def idle(self,ask):
        response = 'n'

        if(not ask):
            time.sleep(2)
            response = 'y'

        if(self.debug_mode):
            response = 'y'

        while(response!='y'):
            response = input("STOP IDLE MODE ? ")
        
    def set_sleep_arm(self,null):
        rospy.loginfo("STATE --> SET ARM TO SLEEP")
        if(not self.debug_mode):
            self.arm_sleep()

    def heat(self,mode):
        if(mode):
            rospy.loginfo("STATE --> SET HEATING PLATE ON")
            self.heater_mode(mode)
            self.heatMODE = mode
        else:
            rospy.loginfo("STATE --> SET HEATING PLATE OFF")
            self.heater_mode(mode)
            self.heatMODE = mode

        
    def goto(self,input):
        gowhere = input[0]
        plan = copy.deepcopy(GOTO_POS[gowhere])
        plan.reverse = input[1] 
        if(not input[1]):
            plan.direction = pi + plan.direction
        
        
        plan.printMD = self.heatMODE

        rospy.loginfo(f"STATE --> GOTO --> {gowhere}")
        rospy.loginfo(f"STATE --> GOTO --> START ACTION")
        self.planner_action.send_goal(plan) # Sends the goal to the action server.
        self.planner_action.wait_for_result()


    def refill(self,null):
        rospy.loginfo("STATE --> CALLING REFILL SERVICE")
        if(not self.debug_mode):
            self.refillBuilder()

    def repeat(self,ask = True):

        if(ask):
            rospy.loginfo(f"DO YOU WANT TO REPEAT STATE {FLOW[self.id-1][0]} ? ")
            response = input("yes or no ? ")
        else:
            response = 'n'

        if(response =='y' or response =='yes' or response==""):
            rospy.loginfo(f"REPEAT STATE {FLOW[self.id-1][0]} ")
            self.id = self.id - 2
            return
        if(response =='n'):
            return
            
        rospy.loginfo("wrong input repeat --> ")
        self.repeat(ask)
        
    def done(self):
        if(len(FLOW)<=self.id):
            return True
        else:
            return False

    def execute_state(self):
        rospy.loginfo(f"STATE --> {self.current_state} --> START")
        self.STATES[self.current_state](self.command)
        rospy.loginfo(f"STATE --> {self.current_state} --> DONE")
        rospy.loginfo("-----------------")
        rospy.loginfo("")
    
    def print_next_state(self):
        if(len(FLOW)>self.id + 1):
            rospy.loginfo(f"NEXT STATE IS --> {FLOW[self.id + 1]}")
        else:
            rospy.logwarn("something is wrong, we run out of stages")

    def next_state(self):
        self.id = self.id + 1
        if(len(FLOW)>self.id):                
            (self.current_state,self.command) = FLOW[self.id]
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
    call service to set arm on sleep position
    """
    def arm_sleep(self):
        rospy.wait_for_service("/px100/go_to_sleep")
        try:
            call_srv = rospy.ServiceProxy("/px100/go_to_sleep", Empty)
            resp1 = call_srv()
            rospy.loginfo("BRAIN  --> SET PX100 TO SLEEP ")
        except rospy.ServiceException as e:
            rospy.logerr("BRAIN  --> Service call failed: %s"%e)

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
    rate = rospy.Rate(2) # publish freacuancy 
    rospy.loginfo("okei lets do it")
    
    # main loop
    while not rospy.is_shutdown():
        if(brain_node.done()):
            break
        
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