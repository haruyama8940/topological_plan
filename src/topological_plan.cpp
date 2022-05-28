#include "ros/ros.h"
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <ros/ros.h>
#include <std_srvs/SetBool.h>          
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/LoadMap.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <yaml-cpp/yaml.h>

class topological_plan
{

public:
    topological_plan(/* args */);
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    bool read_yaml();
    void moving_distance();
    void distanceCallback(std_msgs::Float32 &msg);
private:
    std::string filename_, pass;
    ros::Publisher map_pub;
    ros::Subscriber odom_sub;
};

topological_plan::topological_plan() :
nh_(),
pnh_("~")
{
    pnh_.param("filename", filename_, filename_);
    // odom_sub = nh_.subscribe("odom",&topological_plan::odomCallback,1);

}

bool topological_plan::read_yaml()
{
    YAML::Node yaml_pass= YAML::LoadFile(filename_);
    YAML::Node map_yaml;
    // YAML::Node map_yaml=YAML::LoadFile(filename_);
    map_yaml= yaml_pass["map_list"];
    int i=filename_.size();
    ROS_INFO("size :%d",&i);

    
    return true;
}

void topological_plan::distanceCallback(std_msgs::Float32 &msg)
{
    float distance;
    distance = msg.data
}

int main(int argc, char** argv){
  ros::init(argc, argv, "topological_plan");
 // ros::Rate loop_rate(1);
  topological_plan topo;
  ros::Rate loop_rate(1);
  bool read_result = topo.read_yaml();
    if(!read_result){
        ROS_ERROR("Map changer system is shutting down");
        return 1;
    }
    else{
     while (ros::ok())
       {
            ros::spinOnce();
        // 決められた周期でループするため寝て待つ                                              
            loop_rate.sleep();
       }
    }        
        
    return 0;
    }
