#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
     w    
a    s    d
     x    


s:stop

CTRL-C to quit
"""

moveBindings = {
		'w': [1, 0], 'x': [-1, 0], 
                'a': [0, 1], 'd': [0,-1], 
                's': [0, 0] }


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)  
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 1)
	rospy.init_node('teleop_twist_keyboard')

	x = 0
	z = 0

	try:
		print msg
		
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				z = moveBindings[key][1]

			else:
				
				x = 0
				z = 0
				if (key == '\x03'):
					break

			twist = Twist()

			twist.linear.x = x; twist.angular.z = z 
			
			pub.publish(twist)

	

	finally:
		twist = Twist()
		
		twist.linear.x = 0; twist.angular.z = 0 
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


