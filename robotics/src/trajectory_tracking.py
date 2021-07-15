#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion


class Trajectory_tracking():
    t = []
    x_d = []
    y_d = []
    v_d = []
    w_d = []
    theta_d = []
    q=[]
    dotx_d=[]
    doty_d=[]


    def __init__(self):
        rospy.loginfo("Starting node Trajectory control")
        self.twist_pub = rospy.Publisher('/uv/uv_diffdrive_controller/cmd_vel', Twist, queue_size=10) 

        rospy.Subscriber('/uv/uv_diffdrive_controller/odom',Odometry, self.odometryCb)


    # current robot pose
    def odometryCb(self, msg):
        x = round(msg.pose.pose.position.x,4)
        y = round(msg.pose.pose.position.y,4)
        theta = round(self.get_angle_pose(msg.pose.pose),4) 
        self.q = np.array([x, y, theta])
        return self.q


    # compute angle from quaternion
    def get_angle_pose(self, quaternion_pose):
        q = [quaternion_pose.orientation.x,
                quaternion_pose.orientation.y,
                quaternion_pose.orientation.z,
                quaternion_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        theta = yaw
        return theta


    def cyrcular_trajectory(self, t_max, x_c, y_c):
        t = np.linspace(0, t_max, 1000)
        R = 1
        v_d_val = 0.7           # [m/s], const linear vel
        w_d_val = v_d_val/R     # [rad/s], const angular vel

        y_d = y_c + R * np.cos(w_d_val * t)
        x_d = x_c + R * np.sin(w_d_val * t)

        doty_d = -R*w_d_val*np.sin(w_d_val* t) # time derivative of x_d
        dotx_d =  R*w_d_val*np.cos(w_d_val* t) # time derivative of y_d
        theta_d = np.arctan2(doty_d, dotx_d)

        v_d = np.sqrt(dotx_d**2 + doty_d**2)       
        w_d = w_d_val * np.ones(len(t))

        return [x_d, y_d, v_d, w_d, theta_d, dotx_d, doty_d, t]
        

    def trajectory_generation(self, time, x_c, y_c):
        (self.x_d, self.y_d, self.v_d, self.w_d, self.theta_d, self.dotx_d, self.doty_d, self.t) = self.cyrcular_trajectory(time, x_c, y_c)


    def get_pose(self):
        # get robot position updated from callback
        x = self.q[0]
        y = self.q[1]
        theta = self.q[2]
        return np.array([x, y, theta])


    def get_error(self, T):
        (x, y, theta) = self.get_pose()
        # compute error
        e1 = (self.x_d[T] - x) * np.cos(theta) + (self.y_d[T] - y) * np.sin(theta)
        e2 = -(self.x_d[T] - x) * np.sin(theta) + (self.y_d[T] - y) * np.cos(theta)
        e3 = self.theta_d[T] - theta if len(self.theta_d) else 0
        return np.array([e1, e2, e3])

    
    def get_point_coordinate(self, b):
        # get robot position updated from callback
        x = self.q[0]
        y = self.q[1]
        theta = self.q[2]
        # robot point cooordinate to consider
        y1 = x + b * np.cos(theta)
        y2 = y + b * np.sin(theta)
        return [y1, y2, theta]


    def io_linearization_control_law(self, y1, y2, theta, y1d, y2d, doty1d, doty2d, b):
        # Define the two control gains. Notice we can define "how fast" we track on y_1 and y_2 _independently_
        k_1 = 0.5
        k_2 = 0.5
        
        # return virtual input doty1, doty2
        u_1 = doty1d + k_1*(y1d - y1)
        u_2 = doty2d + k_2*(y2d - y2)

        # return control input v, w
        v = np.cos(theta) * u_1 + np.sin(theta) * u_2
        w = u_2/b * np.cos(theta) - u_1/b *np.sin(theta)

        return np.array([v, w])


    def unicycle_linearized_control(self):
        # Distance of point B from the point of contact P
        b = 0.375

        rospy.sleep(0.1)
        max_t = self.t[len(self.t) - 1]
        len_t = len(self.t)
        for i in np.arange(0, len(self.t)):
            (y1, y2, theta) = self.get_point_coordinate(b)
            # y1 and y2 are inputs of controller
            (v, w) = self.io_linearization_control_law(y1, y2, theta, self.x_d[i], self.y_d[i], self.dotx_d[i], self.doty_d[i], b)
            print("linear:{} and angular:{}".format(v, w))           
            # move robot
            self.send_velocities(v, w, theta)

            print('Errors{}'.format(self.get_error(i)))

            rospy.sleep(max_t/len_t)
        
        self.send_velocities(0,0,0)


    # publish v, w
    def send_velocities(self, v, w, theta=None):
        twist_msg = Twist() # Creating a new message to send to the robot
        
        twist_msg.linear.x = v 
        twist_msg.angular.z = w
        self.twist_pub.publish(twist_msg)
