#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class GoToPose():
    def __init__(self):

        self.goal_sent = False
        # What to do if shut down (e.g. Ctrl-C or failure)
        #rospy.on_shutdown(self.shutdown)
        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))
	
    def goto(self, pos, quat):
        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	    # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        #state = self.move_base.get_state()
        #result = False

        #if success and state == GoalStatus.SUCCEEDED:
            # We made it!
         #   result = True
        #else:
         #   self.move_base.cancel_goal()

        #self.goal_sent = False
        #return result



if __name__ == '__main__':

        rospy.init_node('assign3', anonymous=False)
        navigator = GoToPose()

        # Customize the following values so they are appropriate for your location
        position_1= {'x': 3.85, 'y' : 7.37}
        position_2= {'x': 4.98, 'y' : 3.09}
        position_3= {'x': -3.93, 'y' : 4.31}
        position_4= {'x': -2.55, 'y' : 0.197}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        navigator.goto(position_1,quaternion)
        rospy.sleep(5)

        navigator.goto(position_2,quaternion)
        rospy.sleep(5)

        navigator.goto(position_3,quaternion)
        rospy.sleep(5)

        navigator.goto(position_4,quaternion)
        rospy.sleep(5)
       

    