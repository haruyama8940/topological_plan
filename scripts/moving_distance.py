#!/usr/bin/env python3 
import rospy 
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse
import numpy as np

class distance_diff():
    def __init__(self):
        
        self.odom_x, self.odom_y,self.distance_m = 0.0, 0.0, 0.0
        self.flag =0
        self.branch_flag = False
        self.odom_x_list = np.array([],dtype=float)
        self.odom_y_list = np.array([],dtype=float)
        self.odom_subscriber = rospy.Subscriber('/camera/odom/sample', Odometry, self.callback_odom)
        self.distance_pub = rospy.Publisher('moving_distance',Float32,queue_size=1)
        # self.distance_set = rospy.Service('distance_m',SetBool,callback_branch)
        #self.resp,self.req

    def callback_odom(self,msg):
        odom_x = msg.pose.pose.position.x  
        odom_y = msg.pose.pose.position.y
        self.flag +=1
        if self.flag == 50:
            self.odom_x_list =np.append(self.odom_x_list,odom_x )
            self.odom_y_list =np.append(self.odom_y_list,odom_y)
            self.pub_long =self.distance(self.odom_x_list,self.odom_y_list)
            self.distance_pub.publish(self.pub_long)
            self.flag=0

    def distance(self,x_list, y_list):
        A = np.vstack([x_list, np.ones(len(x_list))]).T
        a,b = np.linalg.lstsq(A,y_list,rcond=None)[0]
        y0 = a*x_list[0] + b
        y_last = a*x_list[-1] + b
        odom_pose_0 = np.append(x_list[0],y0)
        odom_pose_last = np.append(x_list[-1],y_last)
        odom_pose_0 =np.array([x_list[0],y_list[0]])
        odom_pose_last = np.array([x_list[-1],y_list[-1]])
        long = np.linalg.norm(odom_pose_0-odom_pose_last)
        return long


    def callback_branch(self,data):
        resp = SetBoolResponse()
        req = SetBoolRequest()
        if data.data ==True:
            self.odom_x_list = np.zeros
            self.odom_y_list = np.zeros
            self.branch_flag = True
            self.distance_m = data.data
            resp.message = "distance set"
            
if __name__ == '__main__':
    rospy.init_node('moving_distance',anonymous=True)
    diff = distance_diff()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
























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