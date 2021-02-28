#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import curses

role_name = 'brandons_ride'

#---------
#YOUR CODE (Publishers)
manual_throttle_pub = rospy.Publisher('/carla/{}/manual_throttle'.format(role_name), Float32, queue_size=1)
manual_steer_pub = rospy.Publisher('/carla/{}/manual_steer'.format(role_name), Float32, queue_size=1)
# rospy.init_node('keyboard_control_byl24', anonymous=False)
#---------

def keyboard_control():
	stdscr = curses.initscr()
	curses.noecho()
	curses.cbreak()
	stdscr.keypad(1)

	throttle = 0.0
	steer = 0.0

	key = ''
	while key != ord('q'):	#loop until pressing `q`
		stdscr.clear()

		stdscr.addstr(0, 5, "Throttle: %lf" % throttle)
		stdscr.addstr(1, 5, "Steering: %lf" % steer)		
		stdscr.addstr(2, 5, "----------------")
		stdscr.addstr(3, 5, "Spacbar: reset")		
		stdscr.addstr(4, 5, "q: quit")

		key = stdscr.getch()

		#NOTE: throttle and steer increase/decrease by 0.05 when key_up/down/left/right is pressed
		
		if key == curses.KEY_UP:
			throttle += .05
		elif key == curses.KEY_DOWN:
			throttle -= .05
		elif key == curses.KEY_LEFT:
			steer -= .05
		elif key == curses.KEY_RIGHT:
			steer += .05
		elif key == ord(' '): #Spacebar: reset throttle and steer to zero
			throttle = 0
			steer = 0

		#---------
		#YOUR CODE (publish messages)
		manual_throttle_pub.publish(throttle)
		manual_steer_pub.publish(steer)
		#---------

	curses.nocbreak()
	stdscr.keypad(0)
	curses.echo()
	curses.endwin()

if __name__ == '__main__':
	rospy.init_node('keyboard_control_{}'.format('byl24'), anonymous=False)	
	try:
		keyboard_control()
	except rospy.ROSInterruptException:
	        pass
