#!/usr/bin/env python



from numpy.lib.function_base import copy
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
    # print mouth
    # ("GOTO" ,["PREFILL" ,False]),
    # ("GOTO" ,["REFILL" ,False]),
    # ("FILL","NULL"),
    # ("GOTO",["PREPRINT",True]),
    ("GOTO",["CENMOUTH" ,True]),
    # ("HEAT",True),
    ("GOTO",["MOUTH"   ,True]),
    ("INF",False),
    # ("HEAT",False),
    # ("GOTO",["MOUTH"   ,True]),
    # ("INF",False),
    # ("IDLE","NULL"),
    ("GOTO",["PREPRINT",False])]

GOTO_POS = {
"POINT_1": builder.msg.PathPlanInfoGoal(centerX = 1.6, centerY =  0.0  , reverse=True  ,range = 0.1  , init_time = 1   , step_size = 1.0 , mode = "POINT",  direction = 0  , printMD = 0),
"POINT_2": builder.msg.PathPlanInfoGoal(centerX = 1.3, centerY =  0.0  , reverse=False ,range = 0.1  , init_time = 1   , step_size = 1.0 , mode = "POINT",  direction = 0  , printMD = 0),
"LINE":    builder.msg.PathPlanInfoGoal(centerX = 0.4, centerY =  0  , reverse=True ,range = 0.1  , init_time = 1   , step_size = 1.0 , mode = "LINE",  direction = 0  , printMD = 0),
"CIRCLE": builder.msg.PathPlanInfoGoal(centerX = 0.4, centerY =  0  , reverse=True ,range = 0.35  , init_time = 0   , step_size = 4 , mode = "CIRCLE",  direction = 0  , printMD = 0),
"RECT_1": builder.msg.PathPlanInfoGoal(centerX = 0.4, centerY =  0   , reverse=False ,range = 0.1  , init_time = 1   , step_size = 1.0 , mode = "POINT",  direction = 0  , printMD = 0),
"RECT_2": builder.msg.PathPlanInfoGoal(centerX = 0.4, centerY =  0   , reverse=False ,range = 0.1  , init_time = 1   , step_size = 1.0 , mode = "POINT",  direction = 0  , printMD = 0),
"RECT_3": builder.msg.PathPlanInfoGoal(centerX = 0.4, centerY =  0.25 , reverse=False ,range = 0.1  , init_time = 1   , step_size = 1.0 , mode = "POINT",  direction = 0  , printMD = 0),
"RECT_4": builder.msg.PathPlanInfoGoal(centerX = 0.6, centerY =  0.25   , reverse=False ,range = 0.1  , init_time = 1   , step_size = 1.0 , mode = "POINT",  direction = 0  , printMD = 0),
"RECT_5": builder.msg.PathPlanInfoGoal(centerX = 0.4, centerY =  0.25 , reverse=False ,range = 0.1  , init_time = 1   , step_size = 1.0 , mode = "POINT",  direction = -pi/2  , printMD = 0),
"RECT_6": builder.msg.PathPlanInfoGoal(centerX = 0.4, centerY =  0   , reverse=False ,range = 0.1  , init_time = 1   , step_size = 1.0 , mode = "POINT",  direction = 0  , printMD = 0),
"RECT_7": builder.msg.PathPlanInfoGoal(centerX = 0.6, centerY =  0   , reverse=False ,range = 0.1  , init_time = 1   , step_size = 1.0 , mode = "POINT",  direction = 0  , printMD = 0)



}

class BRAIN():

    """ INIT    """
    def __init__(self):
        rospy.init_node('brain',anonymous=True,log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initiating smiley face NODE")

        self.use_real   = rospy.get_param("/use_real")
        self.debug_mode = rospy.get_param("/debug_mode",default=False)

        self.current_state = "START"
        self.command = False
        self.id = 0 
        self.heatMODE = True

        self.STATES = {
            "START":self.start,
            "IDLE":self.idle,
            "HEAT":self.heat,
            "GOTO":self.goto,
            "FILL":self.refill,
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

        
    def idle(self,null):
        response = 'n'

        if(self.debug_mode):
            response = 'y'

        while(response!='y'):
            response = input("STOP IDLE MODE ? ")
        
        
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
            plan.direction = plan.direction + pi
        
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
            response = 'y'

        if(response =='y' or response =='yes' or response==""):
            rospy.loginfo(f"REPEAT STATE {FLOW[self.id-1][0]} ")
            self.id = self.id - 2
            return
        if(response =='n'):
            return
            
        rospy.loginfo("wrong input repeat --> ")
        self.repeat(null = 0)
        

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

        # TEST pt 1-2
        brain_node.goto(["POINT_1",True])
        time.sleep(1)
        brain_node.goto(["POINT_2",False])

        # TEST LINE
        # brain_node.goto([LINE,FALSE])

        # TEST RECT 
        # brain_node.goto(["RECT_1",False])
        # time.sleep(1)
        # brain_node.goto(["RECT_2",False])
        # time.sleep(1)
        # brain_node.goto(["RECT_3",False])
        # time.sleep(1)
        # brain_node.goto(["RECT_4",True])
        # time.sleep(1)


        # brain_node.goto(["RECT_5",False])
        # time.sleep(1)
        # brain_node.goto(["RECT_6",True])
        # time.sleep(1)
        # brain_node.goto(["RECT_7",True])

        # brain_node.goto(["CIRCLE",False])


        rate.sleep()
            
"""  MAIN    """
if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        rospy.logerr("BRAIN --> I THINK NODE DIED...?")