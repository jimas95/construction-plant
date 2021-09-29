import rospy
from arm.candle import CANDLE



def start():

    rospy.init_node('Candle_NODE12', log_level=rospy.DEBUG)
    rate = rospy.Rate(1) # publish freacuancy 
    candle = CANDLE()
    while not rospy.is_shutdown():
        rospy.logdebug("Candle NODE!")
        candle.new_candle()
        rate.sleep()

if __name__ == '__main__':
    try:
        start() 
    except rospy.ROSInterruptException:
        pass
