#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult


status = -1
goal = 1


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


# get goal status
def drug_pose(msg):
    global status
    status = msg.status.status


if __name__ == '__main__':

    rospy.init_node('drug')
    drug_goal = rospy.Publisher('/drug/move_base_simple/goal', PoseStamped, queue_size=1)
    drug1_goal = rospy.Publisher('/drug1/move_base_simple/goal', PoseStamped, queue_size=1)
    rate = rospy.Rate(0.8)

    while not rospy.is_shutdown():
        rospy.Subscriber('/drug/move_base/result', MoveBaseActionResult, drug_pose)
        if goal == 1:
            x_first = 15
            y_first = -6.5
            z_first = 0
            w_first = 1
            result_first = send_goal(x_first, y_first, z_first, w_first)
            drug_goal.publish(result_first) # send first goal

            if status == 3:     # first goal reached
                goal = 2

                rate.sleep()    # to synchronize the status reading by DRUG1

        if goal == 2:
            x_second = 19
            y_second = -16
            z_second = 0
            w_second = 1

            x_drug1 = 19.5
            y_drug1 = -26
            z_drug1 = 0
            w_drug1 = 1
                
            result_second = send_goal(x_second, y_second, z_second, w_second)
            result_drug1 = send_goal(x_drug1, y_drug1, z_drug1, w_drug1)
            rate.sleep()
            drug_goal.publish(result_second) # send second goal
            drug1_goal.publish(result_drug1) # send first goal to DRUG1

        rate.sleep()