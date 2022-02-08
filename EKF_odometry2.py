#!/usr/bin/python2.7


import rospy
import tf
import numpy as np
#import statistics
import time
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from custom_msgs.msg import WheelValues
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

global pub_beta

class State:
    def __init__(self):
        self.x = np.mat(np.zeros(5)).T
        self.y = np.mat(np.zeros(5)).T
        self.z = np.mat(np.zeros(5)).T
        self.u = np.mat(np.zeros(3)).T
        self.B = np.mat(np.zeros(5)).T
        self.Q = np.mat(np.diag([0.05, 0.05, 0.05, 0.0, 0.0]))
        self.R = np.mat(np.diag([10.0, 10.0, 3.0, 10.0, 10.0]))
        self.K = np.mat(np.eye(5))
        self.P = np.mat(np.eye(5))
        self.S = np.mat(np.eye(5))
        self.dt = 0.005
        self.p_delay = 0.0
        self.radius = 0.224  #tire radius
        self.trac = 1.225

class EKF:


    def __init__(self, publisher):
        self.last_msg = None
        self.last_pred = None
        self.state = State()
        self.pub_odom = publisher
        self.pub_beta = publisher

    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)

        return [qx, qy, qz, qw]

    def quaternion_to_euler(self, x, y, z, w):

        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)

        return [X, Y, Z]

    def h(self, x, delay):

        a = x.copy()
        a[0] -= (np.cos(x[2]) * x[3] - np.sin(x[2]) * x[4]) * delay
        a[1] -= (np.cos(x[2]) * x[4] + np.sin(x[2]) * x[3]) * delay
        a[2] -= self.state.u[2] * delay


        return a


    def H(self, x, u, p_delay):
        result = np.mat(np.eye(5))
        result[0, 2] = - (-np.sin(x[2]) * x[3] - np.cos(x[2]) * x[4]) * p_delay
        result[0, 3] = - np.cos(x[2]) * p_delay
        result[0, 4] = - np.sin(x[2]) * p_delay #Erik: wrong! should be positive

        result[1, 2] = - (-np.sin(x[2]) * x[4] + np.cos(x[2]) * x[3]) * p_delay
        result[1, 3] = - np.sin(x[2]) * p_delay
        result[1, 4] = - np.sin(x[2]) * p_delay #Erik: wrong! should be cos not sin

        result[3, 4] = 0.0

        result[4, 3] = 0.0
        return result


    def f(self, x):

        j = x
        j[0] += (np.cos(x[2]) * x[3] - np.sin(x[2]) * x[4]) * self.state.dt
        j[1] += (np.cos(x[2]) * x[4] + np.sin(x[2]) * x[3]) * self.state.dt
        j[2] += self.state.u[2] * self.state.dt
        while j[2] > np.pi:
            j[2] -= 2*np.pi
        while j[2] < -np.pi:
            j[2] += 2 * np.pi

        j[3] = self.state.u[0]
        j[4] = self.state.u[1]
        return j


    def F(self, x, u):
        result = np.mat(np.eye(5))
        result[0, 2] = (-np.sin(x[2]) * x[3] - np.cos(x[2]) * x[4]) * self.state.dt
        result[0, 3] = np.cos(x[2]) * self.state.dt
        result[0, 4] = - np.sin(x[2]) * self.state.dt

        result[1, 2] = (-np.sin(x[2]) * x[4] + np.cos(x[2]) * x[3]) * self.state.dt
        result[1, 3] = np.sin(x[2]) * self.state.dt
        result[1, 4] = np.sin(x[2]) * self.state.dt # Erik: this is wrong! should be cos not sin

        result[3, 4] = 0.0

        result[4, 3] = 0.0
        return result
        #return np.mat(np.eye(5))

    def callback_imu(self, msg):

        self.state.u[2] = msg.angular_velocity.z
        self.state.u[1] = self.state.u[2] * 0.5

    def callback_prediction(self, msg):

        if self.last_pred is not None:

            self.state.dt = (msg.header.stamp - self.last_pred.header.stamp).to_sec()
            #rospy.logwarn(self.state.dt)

        if self.last_msg is None:
            # publish 0 odometry until slam messages are received
            odom = Odometry()
            odom.header.stamp = msg.header.stamp
            odometry.pub_odom.publish(odom)
            return

        #self.state.u[0] = (msg.rr + msg.rl) * self.state.radius / 2
        self.state.u[0] = np.median([msg.rr, msg.rl, msg.fr, msg.fl]) * self.state.radius



        self.state.x = self.f(self.state.x)
        self.state.P = np.dot(np.dot(self.F(self.state.x, self.state.u), self.state.P), self.F(self.state.x, self.state.u).transpose()) + self.state.Q

        # publish odometry output 100 HZ
        #quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, alpha)
        # type(pose) = geometry_msgs.msg.Pose
        quaternion = self.euler_to_quaternion(self.state.x[2], 0.0, 0.0)
        self.last_msg.pose.pose.orientation.x = quaternion[0]
        self.last_msg.pose.pose.orientation.y = quaternion[1]
        self.last_msg.pose.pose.orientation.z = quaternion[2]
        self.last_msg.pose.pose.orientation.w = quaternion[3]

        #self.last_msg.pose.pose.orientation.x = self.state.x[2]
        ##self.last_msg.pose.pose.orientation.y = self.state.y[2]
        #self.last_msg.pose.pose.orientation.z = self.state.z[2]
        #self.last_msg.pose.pose.orientation.w = self.state.u[2]

        self.last_msg.pose.pose.position.x = self.state.x[0]
        self.last_msg.pose.pose.position.y = self.state.x[1]
        self.last_msg.twist.twist.linear.x = self.state.x[3]
        self.last_msg.twist.twist.linear.y = self.state.x[4]
        self.last_msg.twist.twist.angular.z = self.state.u[2]
        beta = np.arctan2(self.state.x[4], self.state.x[3])

        self.last_msg.header.stamp = msg.header.stamp
        self.last_pred = msg

        self.pub_odom.publish(self.last_msg)
        pub_beta.publish(Float64(beta))
        #rospy.loginfo(
            #float(((np.cos(self.state.yaw) * self.state.u + np.sin(self.state.yaw) * self.state.v) * self.dt)))

    def callback_correction(self, msg):


        euler = self.quaternion_to_euler(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        self.state.z[0] = msg.pose.pose.position.x
        self.state.z[1] = msg.pose.pose.position.y
        self.state.z[2] = euler[2]
        self.state.z[3] = msg.twist.twist.linear.x
        self.state.z[4] = msg.twist.twist.linear.y




        if self.last_pred is not None:
            self.state.p_delay = (self.last_pred.header.stamp - msg.header.stamp).to_sec()
            # rospy.logwarn("p_delay = %s", self.state.p_delay)


        self.state.y = self.state.z - self.h(self.state.x, self.state.p_delay)
        while self.state.y[2] > np.pi:
            self.state.y[2] -= 2*np.pi
        while self.state.y[2] < -np.pi:
            self.state.y[2] += 2 * np.pi
        self.state.S = np.dot(np.dot(self.H(self.state.x, self.state.u, self.state.p_delay), self.state.P), self.H(self.state.x, self.state.u, self.state.p_delay).transpose()) + self.state.R
        self.state.K = np.dot(np.dot(self.state.P, self.H(self.state.x, self.state.u, self.state.p_delay).transpose()), np.linalg.inv(self.state.S))
        #self.state.K = np.mat(np.zeros((5, 5)))
        self.state.x += np.dot(self.state.K, self.state.y)
        self.state.P -= np.dot(np.dot(self.state.K, self.H(self.state.x, self.state.u, self.state.p_delay)), self.state.P)


        self.last_msg = msg



if __name__ == "__main__":

    rospy.init_node("EKF_stateestimation", anonymous=True)

    pub_odom = rospy.Publisher("/EKF/odometry", Odometry, queue_size=5)
    pub_beta = rospy.Publisher('/stateestimation/beta', Float64, queue_size=1)
    #pub_odom = rospy.Publisher("/stateestimation/odometry", Odometry, queue_size=5)
    odometry = EKF(pub_odom)

    rospy.Subscriber("/fastslam/odometry", Odometry, odometry.callback_correction, queue_size=5)
    rospy.Subscriber("/stateestimation/wheel_revs_actual", WheelValues, odometry.callback_prediction, queue_size=5)
    rospy.Subscriber("/os1_cloud_node/imu", Imu, odometry.callback_imu, queue_size=5)

    rospy.spin()
