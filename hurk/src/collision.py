#!/usr/bin/env python
import roslib; roslib.load_manifest('hurk')
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

###########################################
#        SENSOR AND ROBOT CLASSES
###########################################
class Sensor:
# Sensor class, holds all sensor attributes: x,y pos, angle, min. dist, and current distance
    def __init__(self, minDist, cushen): #x, y, a, minDist, cushen):
        #self.x = x
        #self.y = y
        #self.a = a
        self.minDist = float(minDist)
        self.cushen = float(cushen) # Distance around minDist to slow down
        self.curDist = -1. # Current distance, is -1 at initalization
    def updateCurDist(self, dist):
    # Update variable curDist
        self.curDist = dist

class Robot:
# Robot class, holds an array of sensors and its center of rotation. Has functions for collision checking and avoidance.
    sensorList = []
    #center
    def __init__(self): #(x,y) , sensList):
        pass
    def checkCollision(self):
        speedMultiplier = 1 # Severity of slowdown. 1=no adjustment 0=full stop.
        for s in self.sensorList:
	    # curDist = -1. at initialization of sensor object, so exclude it if it has not been updated
            if (not (s.curDist < 0)) and (s.curDist < (s.minDist + s.cushen) ):
                sM = max( (s.curDist - s.minDist) / s.cushen , 0 ) # Constrain to >0, negative speed multipliers are not allowed
                speedMultiplier = min(speedMultiplier, sM) # This is to end up with the lowest speed multiplier
        rospy.loginfo( speedMultiplier )
        return speedMultiplier

###########################################
#           CLASS DECLARATIONS
###########################################
#sensorList = []
minDist = 25
cushen = 25
hurk = Robot()
hurk.sensorList.append(Sensor(minDist, cushen)) # Sensor(mindist, cushen)
hurk.sensorList.append(Sensor(minDist, cushen))
hurk.sensorList.append(Sensor(minDist, cushen))
hurk.sensorList.append(Sensor(minDist, cushen))
hurk.sensorList.append(Sensor(minDist, cushen))
#hurk = Robot(sensorList)


###########################################
#             ROS FUNCTIONS
###########################################

def sensCallback(rng):
    n = rng.radiation_type # Quick hack to send sensor number through radiation_type
    hurk.sensorList[n].updateCurDist(rng.range) # Assign range value to sensor's curDist variable
    #rospy.loginfo(rospy.get_name() + " > I heard > %f" % rng.range)
    #Check for collision here
    #Adjust motor speed here
    
def initListener():
    rospy.init_node('collision')
    rospy.Subscriber("range_data", Range, sensCallback)
    #rospy.spin()

def talker():
    pub = rospy.Publisher("slow_down", Float32)
    while not rospy.is_shutdown():
        #str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hurk.checkCollision())
        pub.publish( Float32( hurk.checkCollision() ) )
        rospy.sleep(.05)

if __name__ == '__main__':
    initListener()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

