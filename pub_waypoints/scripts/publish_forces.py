#!/usr/bin/env python
import rospy
from mavros_msgs.msg import ActuatorControl
from geometry_msgs.msg import PoseStamped,TwistStamped
import time
from scipy.spatial.transform import Rotation
import numpy as np
from numpy import linalg as LA
from math import atan, atan2, pi, sin ,cos,fabs,sqrt


pos=[0,0,0]
linVel=[0,0,0]
angVel_body=[0,0,0]

R_cur=np.eye(3)

def cbPos(data):
    global pos,R_cur
    pos = [data.pose.position.x, data.pose.position.y, data.pose.position.z] 
    R_cur=Rotation.from_quat([data.pose.orientation])

def cbVel(data):
    global linVel
    linVel = [data.twist.linear.x, data.twist.linear.y, data.twist.linear.z]

def cbAngVel_body(data):
    global angVel_body
    angVel_body = [data.twist.angular.x, data.twist.angular.y, data.twist.angular.z]

def get_waypoint(t):
	# A = 10 
	# B = 10
	# a = 1
	# b = 2
	x = 0
	y = 0
	z = 5
	vx= 0
	vy= 0
	vz= 0
	ax= 0
	ay= 0
	az= 0
	yaw=0
	dyaw=0
	d2yaw=0
	d3x=0
	d3y=0
	d3z=0
	return [x,y,z,vx,vy,vz,ax,ay,az,yaw,dyaw,d2yaw,d3x,d3y,d3z]   

# def sat_norm(data):
# 	if data>1:
# 		return 1
# 	elif data<0:
# 		return 0
# 	else:
# 		return data		

def sat_norm(data):
	#here the input [throttle, Moment(3dimension)]
	
def controlInput(wp):
	global pos,linVel,angVel_body,R_cur	
	# kp_r=np.array([[0.04],[0.04],[0.04]])	
	#refer http://www-personal.acfr.usyd.edu.au/spns/cdm/papers/Mellinger.pdf
	kp_r=np.diag([0.04,0.04,0.04])	
	kd_r=np.diag([0.2,0.2,0.2])
	# ki_r=np.diag([0.002,0.002,0.002])

	kp_att=np.diag([0.16,0.16,0.16])
	kd_att=np.diag([0.4,0.4,0.4])

	m=1.5
	J=np.diag([0.029125,0.029125,0.05525])
	g=9.81

	x_des=np.array(wp[0:3]).reshape(3,1)
	dx_des=np.array(wp[3:6]).reshape(3,1)
	d2x_des=np.array(wp[6:9]).reshape(3,1)
	d3x_des=np.array(wp[12:15]).reshape(3,1)
	yaw_des=wp[9]
	dyaw_des=wp[10]
	d2yaw_des=wp[11]
	x=np.array(pos).reshape(3,1)
	dx=np.array(linVel).reshape(3,1)
	dAng=np.array(angVel_body).reshape(3,1)

	t=d2x_des+kd.dot(dx_des-dx)+kp.dot(x_des-x)+np.array([0,0,g]).reshape(3,1)
	thrust=m*(t.dot(R_cur))
	Z_des=t/LA.norm(t)
	dummy=np.array([cos(yaw),sin(yaw),0]).reshape(3,1)
	Y_des=(np.cross(Z_des,dummy,axis=0)/LA.norm(np.cross(Z_des,dummy,axis=0)))
	X_des=np.cross(Y_des,Z_des,axis=0)
	R_des=np.append(np.append(X_des,Y_des,axis=1),Z_des,axis=1)
	eR=0.5*(np.transpose(R_des).dot(R_cur)-np.transpose(R_cur).dot(R_des))
	eR=np.array([eR[2][1],eR[0][2]],eR[1][0]).reshape(3,1)

	z_b=(d2x_des+np.array([0,0,g]).reshape(3,1))/LA.norm(d2x_des+np.array([0,0,g]).reshape(3,1))
 	thrust_des=LA.norm(d2x_des+np.array([0,0,g]).reshape(3,1))
 	h_w=(m/thrust_des)*(d3x_des-(np.transpose(z_b).dot(d3x_des))*z_b)

 	w_bw_des=np.array([-np.transpose(h_w).dot(z_b),np.transpose(h_w).dot(X_des),dyaw_des*(np.array([0,0,1]).reshape(3,1)).dot(z_b)])
 	w_bw=np.array(angVel_body).reshape(3,1)
 	M=-kp_att.dot(eR)-kd_att*.dot(w_bw-w_bw_des)
	return [thrust,M[0][0],M[1][0],M[2][0]] 	

def control_loop():
	global pos,linVel

	rospy.init_node('publish_forces')
	pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=100)
	rospy.Subscriber('/mavros/local_position/pose', PoseStamped, cbPos)
	rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, cbVel)
		rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, cbAngVel_body)

	rate = rospy.Rate(40)

	count=1
	wp=get_waypoint(0)
	kp=0.2
	kd=0.01

	cur_force=ActuatorControl() 

	while not rospy.is_shutdown():

		#controlInput
		inp_US=controlInput(wp) #unscale input
		inp_S=sat_norm(inp_US)
		inp_S=inp_S+[0,0,0,0]
		cur_force.group_mix=0

		cur_force.controls=inp_S

		cur_force.header.seq=count
		count=count+1
		cur_force.header.stamp=rospy.get_rostime()
		cur_force.header.frame_id="body_frame"

		pub.publish(cur_force)
		rate.sleep()

if __name__ == '__main__':
    try:
        print("Waiting for gazebo to start")
        time.sleep(5)
        print("Starting the control loop")
        control_loop()
    except rospy.ROSInterruptException:
        pass