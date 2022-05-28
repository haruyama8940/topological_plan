#!/usr/bin/env python3

from torch import long
import rospy
import numpy as np
from nav_msgs.msg import Odometry

def callback_odom(msg):
    global _odom_x, _odom_y, _odon_theta,odom_x_list,odom_y_list # グローバル変数
    odom_x = msg.pose.pose.position.x  # x_pose[m]
    odom_y = msg.pose.pose.position.y  #y_pose[m]
    odom_x_list =np.append(odom_x)
    odom_y_list =np.append(odom_y)

def distance(x_list, y_list):
    a = np.dot(x_list, y_list)/ (x_list ** 2).sum()
    y0 = a * x_list[0]
    y_last = a * x_list[-1]
    odom_pose_0 = np.array(x_list[0],y0)
    odom_pose_last = np.array(x_list[-1],y_last)
    long =  np.absolute(np.linalg.norm(odom_pose_0-odom_pose_last))
    return long

def odometry():
    rospy.init_node('odometry')
    odom_subscriber = rospy.Subscriber('/camera/odom/sample', Odometry, callback_odom)
    moving_distance = rospy.Publisher('/moving_distance',float)
    moving_distance.publish(distance(odom_x_list,odom_y_list))
    rospy.loginfo(long)
    rospy.spin()

if __name__ == '__main__':
    odometry()
