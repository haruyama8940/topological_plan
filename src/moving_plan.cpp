#include "ros/ros.h"
#include <ros/ros.h>
#include <std_srvs/SetBool.h>          
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

class  moving_planner
{
    public:
        moving_planner();
        geometry_msgs::Twist move_vel;
        std_msgs::Float32 moving;
        double IMU_HZ =100.0;
        double target_distance =5.0;
        float rotate_rad = 0.0;
        void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data);
        void distanceCallback(const std_msgs::Float32::ConstPtr& distance);

    private:
        ros::NodeHandle node_;
        ros::Publisher start_vel_pub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber distance_sub_;
        bool stop_flg_;
        double target_yaw_rad_ = 0.0;
        double current_yaw_rad_ = 0.0;
        float current_distance = 0.0;
        float distance =0.0 ;
};

moving_planner::moving_planner(){
    // node_.getParam("moving_planner/IMU_HZ", IMU_HZ);
    // node_.getParam("moving_planner/distance",target_distance);
    start_vel_pub_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel",1,false);
    imu_sub_ = node_.subscribe<sensor_msgs::Imu>("imu_data",1,&moving_planner::imuCallback,this);
    distance_sub_ = node_.subscribe<std_msgs::Float32>("moving_distance", 1, &moving_planner::distanceCallback,this);
    
}

void moving_planner::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data){
    current_yaw_rad_ -=imu_data->angular_velocity.z /IMU_HZ;
    if(!stop_flg_ && target_distance >= current_distance){
        move_vel.linear.x = 0.3;
        move_vel.angular.z = -(target_yaw_rad_ - current_yaw_rad_);
        start_vel_pub_.publish(move_vel);
    }
    else
        ROS_INFO("hai otsu~~!!");
}
void moving_planner::distanceCallback(const std_msgs::Float32::ConstPtr& moving){
    current_distance = moving->data;

}

int main(int argc, char** argv){
    ros::init(argc, argv, "moving_plan");
    moving_planner movingPlanner;
    while(ros::ok()){
        ros::spin();
    }

    return 0;
}