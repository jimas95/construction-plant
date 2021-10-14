#!/usr/bin/env python

import rospy
from arm.candle import CANDLE



def start():

    rospy.init_node('Candle_NODE12', log_level=rospy.DEBUG)
    rate = rospy.Rate(0.5) # publish freacuancy 
    candle_pick  = CANDLE("pick")
    candle_place = CANDLE("place")
    
    candle_pick.new_candle()
    candle_place.new_candle()
    while not rospy.is_shutdown():
        rospy.logdebug("Candle NODE!")
        candle_pick.publish_visualize()
        candle_place.publish_visualize()
        rate.sleep()

if __name__ == '__main__':
    try:
        start() 
    except rospy.ROSInterruptException:
        pass
