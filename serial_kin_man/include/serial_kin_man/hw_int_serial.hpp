#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>


class serial_hardware_interface: public hardware_interface::RobotHW
{
    private:
        ros::Subscriber joint_pos_sub;
        ros::Publisher joint_vel_pub;
        ros::ServiceClient num_jnt_service;
        sensor_msgs::JointState jnt_msg;
        int jnt_num;
        hardware_interface::JointStateInterface jnt_stt_int;
        hardware_interface::VelocityJointInterface vel_jnt_int;

        std::vector<double> pos_;
        std::vector<double> vel_;
        std::vector<double> eff_;
        std::vector<double> cmd_;
        std::vector<std::string> name_;

        
        std::vector<double> callback_data;

    public:
        serial_hardware_interface(ros::NodeHandle& n);
        ~serial_hardware_interface(){};
        void jnt_pos_callback(sensor_msgs::JointState::ConstPtr msg);
        void jnt_vel_pub();
        ros::Duration read();
        void write();
        ros::Time current_time;

};