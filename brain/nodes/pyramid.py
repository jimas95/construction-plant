#!/usr/bin/env python


import rospy
from rospy.core import NullHandler, logerr
from std_srvs.srv import SetBool
from std_srvs.srv import Empty
from brain.srv import myString,myStringRequest,myStringResponse

import actionlib
import builder.msg
import time
from math import pi,cos,sin
import copy

"""
THIS NODE WILL DRAW/PRINT A 2 LINE RAMP
This is a node controling the path planer
Set coolling or printing mode, and where to go print and when to go for refill
"""





plan_the_line = [
    # ("IDLE","NULL"),
    # ("GOTO",["prep__" ,True]),
    # ("GOTO",["cent__" ,True]),
    ("HEAT",True),
    ("GOTO",["line__"   ,True]),
    ("INF",False),
    ("HEAT",False),
    ("GOTO",["prep__" ,False]),
    ("GOTO",["cool__"   ,True]),
    ("INF",False),
    ("GOTO",["prep__",False]),
    ("GOTO",["PREFILL",False]),
    ("IDLE","NULL"),
    ("INF",False),
    ("END","NULL")
]


plan_vertical_line = [
    # ("IDLE","NULL"),
    ("GOTO",["prep_1" ,True]),
    ("GOTO",["cent_1" ,True]),
    ("HEAT",True),
    ("GOTO",["line_1"   ,True]),
    ("INF",False),
    ("HEAT",False),
    ("GOTO",["prep_1" ,False]),
    ("GOTO",["cool_1"   ,True]),
    ("INF",False),
    ("GOTO",["prep_1",False]),
    ("GOTO",["PREFILL",False]),
    ("IDLE","NULL"),
    ("INF",False),
    ("END","NULL")
]






plan_refill = [
    # ("IDLE","NULL"),
    ("GOTO" ,["REFILL"  ,False]),
    ("FILL","NULL"),
    ("GOTO" ,["PREFILL" ,True]),
    ("SLEEP_ARM","NULL"),
    ("IDLE","NULL"),
    ("INF",False),
    ("END","NULL")
]


plan_IDLE = [
    ("IDLE","NULL"),
    ("INF",False),
    ("END","NULL")
]

# WIDTH OF WHEELS       --> 0.16
# LENGTH OFFSET OF FUNS --> 0.35

GOTO_POS = {
# refill points
"REFILL":   builder.msg.PathPlanInfoGoal(centerX = 0.402, centerY = -0.02  , reverse=True ,range = 0.0  , init_time = 1   , step_size = 1.0, mode = "POINT",  direction = 0  , printMD = 0),
"PREFILL":  builder.msg.PathPlanInfoGoal(centerX = 0.5  , centerY = -0.02  , reverse=True ,range = 0.0  , init_time = 1   , step_size = 1.0, mode = "POINT",  direction = 0  , printMD = 0),

# TEMP line
"line__": builder.msg.PathPlanInfoGoal(centerX = 1.3  , centerY =  0.05   , reverse=True ,range = 0.205  , init_time = 0   , step_size = 20  , mode = "LINE",   direction = 0  , printMD = 0),
"cool__": builder.msg.PathPlanInfoGoal(centerX = 0.95  , centerY =  0.05   , reverse=True ,range = 0.205  , init_time = 0   , step_size = 20  , mode = "LINE",   direction = 0  , printMD = 0),
"cent__": builder.msg.PathPlanInfoGoal(centerX = 1.3  , centerY =  0.05   , reverse=True ,range = 0.0  , init_time = 1   , step_size = 1.0 , mode = "POINT",  direction = 0  , printMD = 0),
"prep__": builder.msg.PathPlanInfoGoal(centerX = 1.0   , centerY =  0.05   , reverse=True ,range = 0.0  , init_time = 1   , step_size = 1.0 , mode = "POINT",  direction = 0  , printMD = 0),

# Vertical LINE
"POINT_1":  builder.msg.PathPlanInfoGoal(centerX = 1.15  , centerY = 0.1  , reverse=True ,range = 0.0  , init_time = 1   , step_size = 1.0, mode = "POINT",  direction = pi/2  , printMD = 0),
"POINT_2":  builder.msg.PathPlanInfoGoal(centerX = 1.15  , centerY = 0.32  , reverse=True ,range = 0.0  , init_time = 1   , step_size = 1.0, mode = "POINT",  direction = pi/2  , printMD = 0),


}

class BRAIN():

    """ INIT    """
    def __init__(self):
        rospy.init_node('brain',anonymous=False,log_level=rospy.INFO)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initiating RAMP NODE")
        rospy.Service('brain_mode', myString, self.srv_set_mode)

        self.use_real   = rospy.get_param("/use_real")
        self.debug_mode = rospy.get_param("/debug_mode",default=False)
        self.time = 0 
        self.current_state = "START"
        self.command = False
        self.infinity_mode = True
        self.new_plan = False
        self.id = 0 
        self.current_plan = plan_IDLE
        self.new_name = "None"
        
        self.STATES = {
            "START":self.start,
            "IDLE":self.idle,
            "HEAT":self.heat,
            "GOTO":self.goto,
            "FILL":self.refill,
            "SLEEP_ARM":self.arm_sleep,
            "INF" :self.infinity,
            "END":rospy.signal_shutdown
            }


        # Creates the SimpleActionClient and wait for server to come up
        self.planner_action = actionlib.SimpleActionClient('action_planner', builder.msg.PathPlanInfoAction)
        self.planner_action.wait_for_server()

    def start(self,null):
        rospy.loginfo("INITILIZING 3D PRINTING A RAMP")
        self.heatMODE = False
        self.heat(self.heatMODE)

        
    def idle(self,null):
        time.sleep(0.5)
        

    """
    CALL PATH PLAN ACTION 
    input[0] --> where to go, pull msg from list
    input[1] --> if reverse mode is on (boolean)
    """
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


    def infinity(self,null):
        if(self.infinity_mode):
            rospy.loginfo(f"REPEAT STATE {self.current_plan[self.id-1][0]} ")
            self.id = self.id - 2
            self.update_ramp()


        self.infinity_mode = True


    def update_ramp(self):
# we started with width 2*0.15 and step 30 
# initial offset + 0.05 
# and length 0.2
# 
     #adjust settings so we build a block...?
        self.time = self.time + 1
        y_coord = 0.065*self.time - 0.10
        if(y_coord>0.23):
            self.time = 0 
        GOTO_POS["line__"].centerY = y_coord

    def execute_state(self):
        rospy.loginfo(f"STATE --> {self.current_state} --> START")
        self.STATES[self.current_state](self.command)
        rospy.loginfo(f"STATE --> {self.current_state} --> DONE")
        rospy.loginfo("-----------------")
        rospy.loginfo("")
    

    def next_state(self):
        self.id = self.id + 1
        if(len(self.current_plan)>self.id):                
            (self.current_state,self.command) = self.current_plan[self.id]
        else:
            rospy.logwarn("something is wrong, we run out of stages")
            

    """
    Shut down node
    """
    def shutdown(self):
        rospy.logerr("BRAIN --> SHUT DOWN")


    ############################  SERVICES ############################
    """
    control service for setting modes
    """
    def srv_set_mode(self,input):
        plan = input.mode
        msg_response = myStringResponse()
        self.new_plan = True
        self.new_name = plan
        if(plan=="ll"):
            msg_response.msg =  "Set Plan --> plan temp line"
        elif(plan=="refill"):
            msg_response.msg =  "Set Plan --> plan refill"
        elif(plan=="next"):
            self.infinity_mode = False
            msg_response.msg =  "Moving to next scenario"
        else:
            msg_response.msg =  "wrong input"
        
        return msg_response


    def update_plan(self):
        if(self.new_name=="ll"):
            self.current_plan = plan_the_line
            self.id = 0
        elif(self.new_name=="refill"):
            self.current_plan = plan_refill
            self.id = 0
        elif(self.new_name=="next"):
            self.infinity_mode = False
        else:
            rospy.logerr("wrong input")
            return
        
        (self.current_state,self.command) = self.current_plan[self.id]

    """
    call service to set arm on sleep position
    """
    def arm_sleep(self,null):

        rospy.loginfo("STATE --> SET ARM TO SLEEP")
        if(self.debug_mode):
            return
        
        # call service
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
    def heat(self,mode):
        
        if(mode):
            rospy.loginfo("STATE --> SET HEATING PLATE ON")
        else:
            rospy.loginfo("STATE --> SET HEATING PLATE OFF")

        self.heatMODE = mode

        # call service
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
        
        if(brain_node.new_plan):
            brain_node.new_plan = False
            brain_node.update_plan()

        brain_node.execute_state()
        brain_node.next_state()

        if(brain_node.current_state=="END"):
            rospy.logerr("END OF PLAN... ")
            brain_node.current_plan = plan_IDLE
            brain_node.id = 0 

        rate.sleep()
            
    # # vertical 
    # brain_node.heat(True)
    # time = 0 
    # while not rospy.is_shutdown():
        
    #     brain_node.goto(["POINT_1",False])
    #     brain_node.goto(["POINT_2",True])


    #     time = time + 1
    #     offset_coord = 0.05*time+ 1.15
    #     if(offset_coord>1.2):
    #         time = 0 
    #     GOTO_POS["POINT_1"].centerX = offset_coord
    #     GOTO_POS["POINT_2"].centerX = offset_coord

    #     rate.sleep()


"""  MAIN    """
if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        rospy.logerr("BRAIN --> I THINK NODE DIED...?")