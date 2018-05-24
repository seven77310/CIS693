
By following the “most red” pixel of the ball in the image frames, we observed that robot moves unsmoothly while tracking the ball. To solve that, instead of finding the “most red” pixel in RGB frames, that is a different point in each frame, we extracted the center point of the ball. 
To make the robot turn in the correct direction to track the ball, we assumed a vertical line passing through center of each image frame. Then, in each image frame if the ball's extracted center-point, that represents the ball's position, was in the left side of that line, we set the angular velocity to a positive value to make the robot turn left, and if the ball center position, was in the right side of that line we set the angular velocity to a negative value to make the robot turn right.
By finding the “most red” pixel, while ball is close enough to the camera, the changes in the position of point representing the ball's position in sequential frames, would be big. So, the robot would get diffrent turning commands in small time-steps (e.g. it might get one turn right command, and immidiatly get another turn left command) which causes unsmooth tracking.
To take care of keeping a distance of 1m from the ball, we computed the distance between the robot position and ball center-point. We compared this value with 1 meter to determine if the robot has to go toward the ball or it should go backward. 
At the end of the second video we showed how the robot always consideres the center point of the ball representing the ball's position.

To run the second part of the assignment 3:

First put the ZandG package into catkin workspace and run catkin_make. Then launch "roslaunch turtlebot_bringup minimal.launch --screen". Then, launch RGB topics by entering "roslaunch usb_cam usb_cam­test.launch --screen". Then, launch Depth topics "roslaunch astra_launch astra.launch --screen". and Finally, run "rosrun ZandG ghazal".

Links of Youtube videos:

1. The robot autonomously navigates in the lab to visit the four corner areas:
   https://www.youtube.com/watch?v=jh3LqPsen_A&t=49s

2. The robot follows a ball while maintaining a 1-meter distance:
   https://www.youtube.com/watch?v=O63f4XTL_UQ

