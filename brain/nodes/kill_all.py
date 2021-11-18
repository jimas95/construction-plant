#!/usr/bin/env python


import rospy

"""
This is a node creating a service to kill all activie actions and heating

SERVICE --> kill_all | INPUT Empty msg

"""

from std_srvs.srv import Empty,EmptyRequest, EmptyResponse
from actionlib_msgs.msg import GoalID
from std_srvs.srv import SetBool,SetBoolRequest
import os

class KILL_NODE():


    """ INIT    """
    def __init__(self):
        rospy.init_node('kill_all',anonymous=True,log_level=rospy.INFO)
        rospy.on_shutdown(self.shutdown)
        rospy.Service('/kill_all', Empty, self.kill)
        self.pub_path_plan   = rospy.Publisher('/action_planner/cancel' , GoalID, queue_size=10)
        self.pub_navigation = rospy.Publisher('/action_hunt/cancel'     , GoalID, queue_size=10)

        self.use_real = rospy.get_param("/use_real")
        if(self.use_real):
            rospy.wait_for_service('/heating_node/heatingMode')
            self.close_heat = rospy.ServiceProxy("/heating_node/heatingMode", SetBool)

        
        rospy.loginfo("Initiating Killer NODE")
        rospy.spin()



    """
    service for killing all actions
    """
    def kill(self,EmptyRequest):

        rospy.loginfo("Killing --> brain")
        os.system("rosnode kill /brain")
        rospy.loginfo("Killing --> PathPlan")
        self.pub_path_plan.publish(GoalID())
        rospy.loginfo("Killing --> Navigation")
        self.pub_navigation.publish(GoalID())
        if(self.use_real):
            rospy.loginfo("Killing --> Heating")
            self.close_heat(False)
        
        return EmptyResponse()


    def shutdown(self):
        self.kill
        rospy.logerr("KILLER NODE--> SHUT DOWN")




"""  MAIN    """
if __name__ == '__main__':
    try:
        kill_node = KILL_NODE()
    except rospy.ROSInterruptException:
        rospy.logerr("KILLER --> I THINK NODE DIED...")

