#!/usr/bin/env python


import rospy
from rospy.core import logerr
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse
from builder.srv import PathPlanInfo,PathPlanInfoRequest,PathPlanInfoResponse
from builder.srv import SETMODE,SETMODERequest
from std_srvs.srv import Empty,EmptyRequest,EmptyResponse


from math import pi
"""
THIS NODE WILL DRAW/PRINT A SMILEY FACE
This is a node controling the path planer
Set coolling or printing mode, and where to go print and when to go for refill
"""

MODES = {
        "PRINT_LINE_1": PathPlanInfoRequest(centerX = 1.4, centerY =  0.0   , range = 0.2  , init_time = 0   , step_size = 2.0, mode = "LINE"),
        "COOL_LINE_1":  PathPlanInfoRequest(centerX = 1.4, centerY =  0.0   , range = 0.3  , init_time = 0   , step_size = 2.0, mode = "LINE"),
        "PRINT_LINE_2": PathPlanInfoRequest(centerX = 1.4, centerY =  0.165 , range = 0.2  , init_time = 0   , step_size = 2.0, mode = "LINE"),
        "COOL_LINE_2":  PathPlanInfoRequest(centerX = 1.4, centerY =  0.165 , range = 0.3  , init_time = 0   , step_size = 2.0, mode = "LINE"),
        
        "CIRCLE_1"   :  PathPlanInfoRequest(centerX = 1.5, centerY =  0.2   , range = 0.2   , init_time = 0, step_size = 20 , mode = "CIRCLE"),
        "CIRCLE_2"   :  PathPlanInfoRequest(centerX = 1.5, centerY =  0.2   , range = 0.35  , init_time = 0, step_size = 20 , mode = "CIRCLE"),

        "RESET":        PathPlanInfoRequest(centerX = 0.0, centerY =  0.0   , range = 0.0  , init_time = 0 , step_size = 0.0, mode = "RESET"),
        "REFILL":       PathPlanInfoRequest(centerX = 0.0, centerY =  0.0   , range = 0.0  , init_time = 0 , step_size = 0.0, mode = "REFILL"),
        "STOP"  : SetBoolRequest(True),
        "RESUME": SetBoolRequest(False),
        }




FLOW = [
    ("IDLE" ,"NULL"),
    # ("GOTO" ,["PREPRINT",True]),
    # ("GOTO" ,["REFILL"  ,False]),
    # ("FILL","NULL"),
    # ("GOTO",["PREPRINT",True]),
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
    ("END","NULL")]

GOTO_POS = {
"REFILL":   PathPlanInfoRequest(centerX = 0.25, centerY =  0.0   , reverse=True ,range = 0.0  , init_time = 0   , step_size = 1.0, mode = "POINT"),
"PREPRINT": PathPlanInfoRequest(centerX = 0.5 , centerY =  0.0   , reverse=True ,range = 0.0  , init_time = 0   , step_size = 1.0, mode = "POINT"),
"EYE_1":    PathPlanInfoRequest(centerX = 1.0 , centerY =  0.0   , reverse=True ,range = 0.0  , init_time = 0   , step_size = 1.0, mode = "POINT"),
"EYE_2":    PathPlanInfoRequest(centerX = 1.0 , centerY =  0.0   , reverse=True ,range = 0.0  , init_time = 0   , step_size = 1.0, mode = "POINT")
}

class BRAIN():

    """ INIT    """
    def __init__(self):
        rospy.init_node('brain',anonymous=True,log_level=rospy.DEBUG)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Initiating smiley face NODE")

        rospy.Service('/brain_node/setMode', SETMODE, self.setMode)
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
        self.changePlan(plan)

    def refill(self,null):
        rospy.loginfo("STATE --> CALLING REFILL SERVICE")
        self.refillBuilder()


    def execute_state(self):
        rospy.loginfo(f"STATE --> {self.current_state} --> START")
        self.STATES[self.current_state](self.command)
        rospy.loginfo(f"STATE --> {self.current_state} --> DONE")
        rospy.loginfo("-----------------")
        rospy.loginfo("")
    
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

    def setMode(self,mode_msg):
        self.mode = mode_msg.mode

        if(self.mode =="STOP"): # STOP PLANNER (NOT CMD VEL)
            self.heater_mode(False)
            self.stopPlan(MODES["STOP"])

        elif(self.mode=="RESUME"): # RESUME PLANNER
            self.stopPlan(MODES["RESUME"])

        elif(self.mode=="PRINT_LINE_1"):
            # self.heater_mode(True)
            self.changePlan(MODES["PRINT_LINE_1"])

        elif(self.mode=="PRINT_LINE_2"):
            # self.heater_mode(True)
            self.changePlan(MODES["PRINT_LINE_2"]) 

        elif(self.mode=="COOL_LINE_1"):
            # self.heater_mode(False)
            self.changePlan(MODES["COOL_LINE_1"])

        elif(self.mode=="COOL_LINE_2"):
            # self.heater_mode(False)
            self.changePlan(MODES["COOL_LINE_2"]) 
        
        elif(self.mode=="REFILL"):  
            self.changePlan(MODES["REFILL"]) 
            # self.refillBuilder() 

        elif(self.mode=="RESET"):
            self.changePlan(MODES["RESET"]) 

        elif(self.mode=="CIRCLE_1"):
            self.changePlan(MODES["CIRCLE_1"]) 

        elif(self.mode=="CIRCLE_2"):
            self.changePlan(MODES["CIRCLE_2"]) 

        elif(self.mode=="---"):
            pass
        elif(self.mode=="---"):
            pass
        elif(self.mode=="---"):
            pass
        elif(self.mode=="---"):
            pass
        else:
            rospy.logerr("BRAIN --> ERROR ")
            rospy.logerr("BRAIN --> no mode found...?")
            rospy.logerr("BRAIN --> mode:" + self.mode)
            rospy.logerr("BRAIN --> ERROR ")

        return("BRAIN --> SETTING MODE : " + self.mode)


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
    change path planing node modes or settings 
    """
    def changePlan(self,mode):
        rospy.wait_for_service('/path_plan/set_plan')
        try:
            call_srv = rospy.ServiceProxy('/path_plan/set_plan', PathPlanInfo)
            resp1 = call_srv(mode)
            rospy.loginfo("SET PLAN --> ********* " + resp1.msg)
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
    rate = rospy.Rate(1) # publish freacuancy 
    rospy.loginfo("okei lets do it")
    
    # main loop
    while not rospy.is_shutdown():
        brain_node.execute_state()
        brain_node.print_next_state()
        input()
        brain_node.next_state()
        rate.sleep()
            
"""  MAIN    """
if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        rospy.logerr("BRAIN --> I THINK NODE DIED...?")