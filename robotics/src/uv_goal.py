#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from trajectory_tracking import Trajectory_tracking


uv_circ = 1


# send navigation goal pose
def send_goal(x, y, z, w):
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = z
    goal.pose.orientation.w = w
    return goal


if __name__ == '__main__':

    rospy.init_node('uv')
    uv_goal = rospy.Publisher('/uv/move_base_simple/goal', PoseStamped, queue_size=1)
    rate = rospy.Rate(0.8)

    while not rospy.is_shutdown():

        if uv_circ == 1:                    
            tt=Trajectory_tracking()
            tt.trajectory_generation(5, 2, -0.5)
            tt.unicycle_linearized_control()
            uv_circ = 0   # for following the circular trajectory only one time
            rospy.loginfo("UV circular trajectory done!")

        x_first = 19
        y_first = -28
        z_first = 0.7
        w_first = 0.7
        result = send_goal(x_first, y_first, z_first, w_first)
        rate.sleep()
        uv_goal.publish(result)
