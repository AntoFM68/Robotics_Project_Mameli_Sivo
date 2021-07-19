#!/usr/bin/env python  
import rospy
import math
import tf2_ros
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult


max_drug_angular = 2.5
max_drug_linear = 2.5
min_distance = 1.5
x_vel_drug = 0
z_vel_drug = 0
status = -1
goal = 1


# get v and w
def drug_vel(msg):
    global x_vel_drug
    global z_vel_drug
    x_vel_drug = msg.linear.x
    z_vel_drug = msg.angular.z


# get goal status
def drug_pose(msg):
    global status
    status = msg.status.status

if __name__ == '__main__':
    rospy.init_node('tf2_drug_listener')
 
    tfBuffer = tf2_ros.Buffer()

    drug_velocity = rospy.Publisher('/drug1/drug1_diffdrive_controller/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("drug1_base_footprint", 'drug_base_footprint', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        rospy.Subscriber('/drug/drug_diffdrive_controller/cmd_vel', Twist, drug_vel)
        rospy.Subscriber('/drug/move_base/result', MoveBaseActionResult, drug_pose)

        if (x_vel_drug == 0 and z_vel_drug == 0):
            print('Waiting for DRUG start')
        else:
            # DRUG robot is moving, DRUG1 robot is following
            msg = geometry_msgs.msg.Twist()
            distance = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

            if distance > min_distance and goal == 1:
                msg.angular.z = min(4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x), max_drug_angular)
                msg.linear.x = min(0.5 * distance, max_drug_linear)
            elif status == 3:
                # Goal reached
                goal = 2
                msg.angular.z = 0
                msg.linear.x = 0
            elif distance < min_distance:
                # Too close, STOP robot
                msg.angular.z = 0
                msg.linear.x = 0
            drug_velocity.publish(msg)

        rate.sleep()
