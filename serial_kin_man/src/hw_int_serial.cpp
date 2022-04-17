#include "../include/serial_kin_man/hw_int_serial.hpp"
#include <sstream>
#include <hardware_interface/robot_hw.h>
#include "kinematic_simulator/num_jnt.h"

serial_hardware_interface::serial_hardware_interface(ros::NodeHandle& n)
{
    int res,num_jnt,i;
    std::string s;
    KDL::Tree robot_tree;
    KDL::SegmentMap robot_seg_map;
    std::ostringstream ss;
    kinematic_simulator::num_jnt srv;
    this->joint_vel_pub = n.advertise<sensor_msgs::JointState>("kin_sim/joint_vel_",5);
    this->joint_pos_sub = n.subscribe(
        "kin_sim/joint_pos",
        5,
        &serial_hardware_interface::jnt_pos_callback,
        this
        );

    this->num_jnt_service = n.serviceClient<kinematic_simulator::num_jnt>("number_jnt");
    if(!this->num_jnt_service.call(srv))
    {
        ROS_ERROR("Error in service request");
    }
    num_jnt = srv.response.jnt_num;
    /*if(n.getParam("robot_description",s))
        ROS_INFO("exists parameter <robot_description> and its value is %s", s.c_str());
    else
        ROS_WARN("Hardware Interface Failed getParam");
    
    res = kdl_parser::treeFromString(s,robot_tree);
    if(!res)
        ROS_WARN("Faild Parsing of Robot Tree");
    num_jnt = robot_tree.getNrOfJoints();*/
    ROS_INFO("Number of joint is %d",num_jnt);

    //resize all vector and JointState msgs with number of joint 
    this->pos_.resize(num_jnt);
    this->vel_.resize(num_jnt);
    this->eff_.resize(num_jnt);
    this->cmd_.resize(num_jnt);
    this->callback_data.resize(num_jnt);

    this->jnt_msg.velocity.resize(num_jnt);
    this->jnt_msg.name.resize(num_jnt);

    
    //build hardware interface
    for(i=0;i<num_jnt;i++)
    {
        this->pos_[i] = 0;
        this->vel_[i] = 0;
        this->eff_[i] = 0;
        this->callback_data[i] = 0;
        ss << "jnt_"<< i+1;
        s = ss.str();
        ROS_WARN("%s",s.c_str());
        this->jnt_msg.name[i]=s;
        hardware_interface::JointStateHandle state_handle(
            ss.str(),
            &this->pos_[i],
            &this->vel_[i],
            &this->eff_[i]
        );
        this->jnt_stt_int.registerHandle(state_handle);

        hardware_interface::JointHandle vel_handle(
            this->jnt_stt_int.getHandle(s),
            &this->cmd_[i]
        );
        this->vel_jnt_int.registerHandle(vel_handle);
        ss.str("");
        ss.clear();
    }
    //registerInterface(&this->jnt_stt_int);
    registerInterface(&this->jnt_stt_int);
    registerInterface(&this->vel_jnt_int);
    this->jnt_num = num_jnt;
    this->current_time = ros::Time::now();
    
    

}
void serial_hardware_interface::jnt_pos_callback(sensor_msgs::JointState::ConstPtr msg)
{
    int i;
    for(i=0;i<jnt_num;i++)
    {
        //ROS_INFO("il valore del giunto %d è %f",i,msg->position[i]);
        this->callback_data[i] = msg->position[i];
    }
}

void serial_hardware_interface::jnt_vel_pub()
{
    int i;
    for(i=0;i<jnt_num;i++)
    {
        this->jnt_msg.velocity[i]=1;
    }
    this->joint_vel_pub.publish(this->jnt_msg);
}
ros::Duration serial_hardware_interface::read() 
{
    int i;
    ros::Time new_time;
    ros::Duration time_diff;
    for(i=0;i<this->jnt_num;i++)
    {
        this->pos_[i] = this->callback_data[i];
    }
    new_time = ros::Time::now();
    time_diff = new_time - this->current_time;
    this->current_time = new_time;
    return time_diff;
    
}

void serial_hardware_interface::write()
{
    int i;
    for(i=0;i<this->jnt_num;i++)
    {
        this->jnt_msg.velocity[i] = this->cmd_[i];
    }
    this->joint_vel_pub.publish(this->jnt_msg);
}