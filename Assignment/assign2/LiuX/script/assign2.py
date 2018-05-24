#!/usr/bin/env python
import roslib
import rospy
import turtlesim.srv as t_srv
import random
import math
import std_srvs.srv as std_srv
import turtlesim.msg as t_msg
import geometry_msgs.msg as g_msg


def runner_turtle():
	x = random.randint(1, 10)
	y = random.randint(1, 10)
	theta = random.uniform(-math.pi, math.pi)
	create_turtle(x, y, theta,"runner")

def hunter_turtle():
	x = random.randint(1, 10)
	y = random.randint(1, 10)
	theta = random.uniform(-math.pi, math.pi)
	create_turtle(x, y, theta,"hunter")

def runner_motion():
	run = g_msg.Twist()
	run.linear.x = 1.0
	run.linear.y = 0.0
	run.linear.z = 0.0
	run.angular.x = 0.0
	run.angular.y = 0.0
	run.angular.z = random.uniform(-1, 1)
	
	runner_pub = rospy.Publisher("/runner/cmd_vel", g_msg.Twist, queue_size = 10)
	runner_pub.publish(run)

def r_callback(msg):
	global r_pose
	r_pose=msg

def h_callback(msg):
	global h_pose
	h_pose=msg
	

def hunter_motion():
	chase = g_msg.Twist()
	chase.linear.x = 1.0
	chase.linear.y = 0.0
	chase.linear.z = 0.0
	chase.angular.x = 0.0
	chase.angular.y= 0.0
	chase.angular.z=math.atan2(r_pose.y - h_pose.y, r_pose.x - h_pose.x)-h_pose.theta
	hunter_pub =rospy.Publisher("/hunter/cmd_vel", g_msg.Twist, queue_size = 10)
	hunter_pub.publish(chase)




if __name__ == "__main__":
	rospy.init_node('assign2')
	time = rospy.Rate(0.5)

	kill_turtle = rospy.ServiceProxy('/kill',t_srv.Kill) #Delets a turtle
	kill_turtle("turtle1")
	create_turtle=rospy.ServiceProxy('/spawn',t_srv.Spawn)

	runner_turtle()
	r_pose = t_msg.Pose()

	hunter_turtle()
	h_pose = t_msg.Pose()	
	while not rospy.is_shutdown():
		#runner runs
		runner_motion()
		rospy.Subscriber("/runner/pose", t_msg.Pose, r_callback)
		rospy.Subscriber("/hunter/pose",t_msg.Pose,h_callback)

		dist= math.sqrt((h_pose.x - r_pose.x)**2+(h_pose.y - r_pose.y)**2)
		if dist < 1:
			kill_turtle("runner")
			clear_backg = rospy.ServiceProxy('/clear', std_srv.Empty)
			clear_backg()
			runner_turtle()
			r_pose = t_msg.Pose()

		else:
			hunter_motion()
		time.sleep()

		


#	x = random.randint(1, 10)
#	y = random.randint(1, 10)
#	theta = random.uniform(-math.pi, math.pi)
#	create_turtle(x, y, theta,"runner")
#	runner_pos=x,y,theta
	
	
	

