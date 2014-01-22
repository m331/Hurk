#!/usr/bin/env python
import roslib; roslib.load_manifest('hurk')
import rospy
from sensor_msgs.msg import Joy # For joystick input
#from geometry_msgs.msg import Twist # For base velocity output
from std_msgs.msg import Float32MultiArray # For arm velocity output

# Defining ps3 controller buttons and speeds
UPPERARM = {'up':      10, 'down': 8, 'speed': 1} # buttons
LOWERARM = {'up':      11, 'down': 9, 'speed': 1} # buttons
HIP      = {'up':       4, 'down': 6, 'speed': 1} # Buttons
BASE     = {'straight': 1, 'turn': 0, 'straightspeed': 1, 'turnspeed': 1} # Axes
GRIPPER  = 14 # Gripper button
#STOP     = 15 # TODO: Implement stop button
# TODO: Define limits in dictionary above

class RobotState:
	# a in this context means alpha which means angle.
	# All angles are defined using the standard coordinate system.
	#a_knee, a_hip, a_shoulder, a_elbow = 0,0,0,0
	controlState = {
		'upperarm': 0.0,
		'lowerarm': 0.0,
		#'base'    : {'straight': 0, 'turn': 0},
		'hip'     : 0.0
		'gripper' : 0
	}
	# Define measurements
	# Todo: define limits?
	def __init__(self):
		# initialize pub/sub
		rospy.init_node('PS3toMOVEMENT')
		#rospy.Subscriber("sometopic", #sensor msg type# Range, #callback function# sensCallback)
		# Publisher that publishes a Twist msg array
		self.jointspub = rospy.Publisher('joints_vel', Float32MultiArray)

	#def setAngles(self, a_knee, a_hip, a_shoulder, a_elbow):
	#	self.a_knee     = a_knee
	#	self.a_hip      = a_hip
	#	self.a_shoulder = a_shoulder
	#	self.a_elbow    = a_elbow

	# PS3 controller callback, 
	def joyCall(self, joymsg):
		axe = joymsg.axes
		btn = joymsg.buttons
		print('debug, axes: ' + axe + ' btn:' + btn)
		controlState['upperarm'] = (btn[UPPERARM['up']] - btn[UPPERARM['down']]) * UPPERARM['speed']
		controlState['lowerarm'] = (btn[LOWERARM['up']] - btn[LOWERARM['down']]) * LOWERARM['speed']
		controlState['hip']      = (btn[     HIP['up']] - btn[     HIP['down']]) *      HIP['speed']
		controlState['gripper']  =  btn[GRIPPER]
		#controlState['base']['straight'] = axe[BASE['straight']] * BASE['straightspeed']
		#controlState['base']['turn']     = axe[BASE['turn']]     * BASE['turnspeed']
		sendControl()

	# Sends joint velocities to control node
	def sendControl(self):
		cS = controlState
		msg = Float32MultiArray
		msg.data = [ cS['upperarm'], cS['lowerarm'], cS['hip'], cS['gripper'] ]
		self.jointspub.publish( msg )
		# There must be a timeout function in the receiving code
		# for when the joystick fails and this function doesn't get called any more.

	def updateState(self):
		# Callback that reads out the odometer data
		pass

	def checkExtremes(self):
		# Callback that checks if we're near the joint extremes
		pass

	def checkCollisions(self):
		# Checks if arms collide with floor or robot itself.
		pass


if __name__ == '__main__':
	hurk = Robot() # Robot's __init__ initializes pub/sub automatically