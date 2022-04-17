#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/treefksolver.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/segment.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include "../include/kin_sim_class.hpp"
#include "kinematic_simulator/num_jnt.h"

int joint;
void subcribe_joint_vel_call(const kinematic_simulator::joint_vel::ConstPtr &msg, int jnt_id , kin_sim_class obj)
{   
    //ROS_INFO("joint %d has recived %f velocity command", jnt_id,msg->value);
    obj.set_joint_vel(jnt_id,msg->value);
    
}
bool joints_(kinematic_simulator::num_jnt::Request& req,kinematic_simulator::num_jnt::Response &res)
{
    res.jnt_num = joint;
    return true;
}



int main(int argc,char** argv)
{
    ros::init(argc,argv,"kin_sim");
    
    std::string s;
    KDL::Tree my_tree;
    KDL::Frame ee_frame;
    KDL::SegmentMap prova_sg;
    KDL::Joint  jnt__;
    KDL::JntArray q_;
    KDL::Segment sgmnt__;
    bool res;
    int jnt_num,i;
    ROS_INFO("Start Thread");
   
    ros::init(argc,argv,"joint_publisher");
    ros:: NodeHandle n;
    ros::Rate loop_rate(50);
    kin_sim_class prova(n);
    
    
    joint = jnt_num = prova.get_joint_size(true);

    ros::ServiceServer jnt_srv = n.advertiseService("number_jnt",joints_);
    q_.resize(jnt_num);
    q_(0,0) = 0;
    q_(1,0) = 0;
    q_(2,0) = 0;
    //init subscriber
  
    prova.init(q_);
    while (ros::ok())
    {
        prova.update_q();
        prova.publish_joint();

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
