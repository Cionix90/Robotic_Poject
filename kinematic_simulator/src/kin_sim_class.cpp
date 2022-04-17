#include <ros/ros.h>
#include "../include/kin_sim_class.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <sensor_msgs/JointState.h>
#include <kinematic_simulator/joint_vel.h>

#include <sstream>
#include "kinematic_simulator/num_jnt.h"

//completa creazione pub e sub per poi provare questa classe
kin_sim_class::kin_sim_class(ros::NodeHandle &node)
{
    std::string s;
    std::stringstream ss;
    int num_jnt, res,i;
    KDL::Tree robot_tree;
    KDL::SegmentMap robot_seg_map;
    if(node.getParam("robot_des",s))
        ROS_INFO("exists parameter <robot_description> and its value is %s", s.c_str());
    else
        ROS_WARN("Failed getParam");
    
    res = kdl_parser::treeFromString(s,robot_tree);
    if(!res)
        ROS_WARN("Faild Parsing of Robot Tree");
    num_jnt = robot_tree.getNrOfJoints();

    //this->pub_jnt = node.advertise<sensor_msgs::JointState>("joint_states",5);
    this->pub_control_jnt = node.advertise<sensor_msgs::JointState>("kin_sim/joint_pos",5);
    //ros::Subscriber sub_ee_frame =node.subscribe("ee_frame",5,)

    //resize JointState and init Joint Name
    jointState.name.resize(num_jnt);
    jointState.position.resize(num_jnt);
    //jointState.velocity.resize(num_jnt);
    //jointState.effort.resize(num_jnt);

    robot_seg_map = robot_tree.getSegments();
    KDL::SegmentMap::const_iterator it = robot_seg_map.begin();
    //salto il segmento che rappresenta il giunto
    it++;
    this->joint_ = KDL::JntArray(num_jnt);
    this->joint_vel_ = KDL::JntArray(num_jnt);
    this->sub_jnt_vel = node.subscribe<sensor_msgs::JointState>(
        "kin_sim/joint_vel_",
        5,
        &kin_sim_class::callback_update_vel,
        this
        );
    for(i=0;i<num_jnt;i++)
    {
        this->jointState.name[i]=it->second.segment.getJoint().getName();
        this->jointState.position[i] = 0;
        
        ROS_INFO("%d joint name is %s",i,jointState.name[i].c_str());
        it++;
    }
    this->jnt_num = num_jnt;
    //this->joints_srv = node.advertiseService("jnt_service",&kin_sim_class::joints_,this);

    
}
int kin_sim_class::get_joint_size(bool deb)
{
    if(deb)
        ROS_INFO("number of robot joint is %d",this->joint_.rows());
    return joint_.rows();
}
bool kin_sim_class::init(KDL::JntArray init_pos)
{
    int i,n = joint_.rows();
    
    if(this->joint_.rows() != init_pos.rows())
    {
        ROS_WARN("Failed kin_sim_class init; wrong input dimesions");
        return false;
    }
    this->joint_ = init_pos;
    KDL::SetToZero(this->joint_vel_);
    for(i=0;i<n;i++)
    {
        ROS_INFO("%d value of joint pos is %f",i,joint_(i,0));
    }
    this->old_time = ros::Time::now();
    return true;
}
void kin_sim_class::publish_joint()
{
    for(int i = 0;i< this->get_joint_size(false);i++)
    {
        this->jointState.position[i] = this->joint_(i,0);
        this->jointState.header.stamp = ros::Time::now();
    }
    //pub_jnt.publish(jointState);
    pub_control_jnt.publish(jointState);
    
}

void kin_sim_class::set_joint_vel(int index,double value)
{
    this->joint_vel_(index,0) = value;
    ROS_WARN("joint vel is [%f,%f]",joint_vel_(0,0),joint_vel_(1,0));
}
void kin_sim_class::callback_update_vel(const sensor_msgs::JointState::ConstPtr &msg)
{
    int n = this->get_joint_size(false),i;
    for(i=0;i<n;i++)
    {
        this->joint_vel_(i,0) = msg->velocity[i];
    }
    
}
void kin_sim_class::update_q()
{
    int i,n = this->get_joint_size(false);
    ros::Time new_time = ros::Time::now();
    ros::Duration dt = new_time - old_time;
    old_time = new_time;
    //ROS_INFO("update joints from [%f,%f]",joint_(0,0),joint_(1,0));
    for(i = 0; i < n ;i++)
    {
        //ROS_INFO("%d esimo joint of %d vel is %f",i,n,joint_vel_(i,0));
        this->joint_(i,0) = this->joint_(i,0) + this->joint_vel_(i,0)*dt.toSec();
        
    }
    //ROS_INFO("TO [%f,%f]",joint_(0,0),joint_(1,0));
}
