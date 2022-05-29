#!/usr/bin/env python3 
from pygments import lex
import rospy 
from nav_msgs.msg import Odometry
import numpy as np
import tf
from tf.transformations import euler_from_quaternion

odom_x, odom_y = 0.0, 0.0
flag =0
odom_x_list = np.array([],dtype=float)
odom_y_list = np.array([],dtype=float)

def distance(x_list, y_list):
    # A = np.array([x_list,np.ones(len(x_list))])
    # A = A.T
    # a,b = np.linalg.lstsq(A,y_list)[0]
    # y0 = a*x_list[0] + b
    # y_last = a*x_list[-1] + b
    # odom_pose_0 = np.append(x_list[0],y0)
    # odom_pose_last = np.append(x_list[-1],y_last)
    odom_pose_0 =np.array([x_list[0],y_list[0]])
    odom_pose_last = np.array([x_list[-1],y_list[-1]])
    long = np.linalg.norm(odom_pose_0-odom_pose_last)
    return long

def callback_odom(msg):
    global odom_x, odom_y, odom_x_list,odom_y_list,flag
    odom_x = msg.pose.pose.position.x  
    odom_y = msg.pose.pose.position.y
    flag +=1
    # print(flag)
    if flag == 50:
        odom_x_list =np.append(odom_x_list,odom_x )
        odom_y_list =np.append(odom_y_list,odom_y)
        flag=0
        pub_long =distance(odom_x_list,odom_y_list)
        print(pub_long ,"[m]")
    # rospy.loginfo("Odomery: x=%s y=%s", odom_x_list, odom_y_list,)
    # rospy.loginfo("x_shape = %s, y_shape =%s" ,len(odom_x_list),len(odom_y_list))


# def callback_branch(msg):
#     if msg.data ==True:
#         odom_x_list = np.zeros
#         odom_y_list = np.zeros

def odometry():
    rospy.init_node('odometry')
    odom_subscriber = rospy.Subscriber('/camera/odom/sample', Odometry, callback_odom)
    rospy.loginfo("start")
    rospy.spin()

if __name__ == '__main__':
    odometry()























# #!/usr/bin/env python3

# import rospy
# import numpy as np
# from nav_msgs.msg import Odometry

# def callback_odom(msg):
    
#     odom_x = msg.pose.pose.position.x  # x_pose[m]
#     odom_y = msg.pose.pose.position.y  #y_pose[m]
#     # odom_x_list =np.append(odom_x)
#     # odom_y_list =np.append(odom_y)

#     rospy.loginfo(odom_x,odom_y)
# # def distance(x_list, y_list):
# #     A = np.array([x_list,np.ones(len(x_list))])
# #     A = A.T
# #     a,b = np.linalg.lstsq(A,y_list)[0]
# #     y0 = a*x_list[0] + b
# #     y_last = a*x_list + b
# #     odom_pose_0 = np.array(x_list[0],y0)
# #     odom_pose_last = np.array(x_list[-1],y_last)
# #     long =  np.absolute(np.linalg.norm(odom_pose_0-odom_pose_last))
# #     return long

# # def odometry():
    
# #     # # moving_distance = rospy.Publisher('/moving_distance',float)
# #     # # moving_distance.publish(distance(odom_x_list,odom_y_list))
# #     # if len(odom_x_list)>=10:    
# #     #     distance(odom_x_list,odom_y_list)
# #     #     rospy.loginfo(long)
# #     rospy.spin()

# if __name__ == '__main__':
#     # odometry()
#     rospy.init_node('odometry_py')
#     odom_subscriber = rospy.Subscriber('/camera/odom/sample', Odometry, callback_odom)
#     rospy.spin()