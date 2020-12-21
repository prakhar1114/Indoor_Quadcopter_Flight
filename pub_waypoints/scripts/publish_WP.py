#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from math import sin, cos
import time

pos=[0.0, 0.0, 0.0]

def get_waypoint(t):
	A = 10 
	B = 10
	a = 1
	b = 2
	x = A*cos(a*t)-3
	y = B*sin(b*t)
	z = 1
	return [x,y,z] 

def callback(data):
    global pos
    pos = [data.pose.position.x, data.pose.position.y, data.pose.position.z]    

def control_loop():
	global pos
	rospy.init_node('publish_WP')
	pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=100)
	rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback)
	rate = rospy.Rate(10)

	i = 0
	wp= get_waypoint(i)
	dist_error = 100  # some random value
	count =1
	while not rospy.is_shutdown():
    
		## When close to waypoint, sample a new waypoint
		if dist_error < 1:
		    i = i+0.2
		    wp = get_waypoint(i)    

		### Compute errors
		x_err = wp[0]-pos[0]
		y_err = wp[1]-pos[1]
		z_err = wp[2]-pos[2]
		dist_error = (x_err**2 + y_err**2+ z_err**2)**0.5

		cur_WP=PoseStamped() 
		cur_WP.pose.position.x=wp[0]
		cur_WP.pose.position.y=wp[1]
		cur_WP.pose.position.z=wp[2]

		cur_WP.pose.orientation.x=0
		cur_WP.pose.orientation.y=0
		cur_WP.pose.orientation.z=0
		cur_WP.pose.orientation.w=-1

		cur_WP.header.seq=count
		count=count+1
		cur_WP.header.stamp=rospy.get_rostime()
		cur_WP.header.frame_id="world_NED"

		pub.publish(cur_WP)
		rate.sleep()

if __name__ == '__main__':
    try:
        print("Waiting for gazebo to start")
        time.sleep(5)
        print("Starting the control loop")
        control_loop()
    except rospy.ROSInterruptException:
        pass