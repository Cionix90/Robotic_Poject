#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chaindynparam.hpp>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <boost/scoped_ptr.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <string>
#include <kdl/jntarrayacc.hpp>
namespace rrr_dynamic_controller
{
    class Computed_Torque_controller : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
            Computed_Torque_controller();
            ~Computed_Torque_controller();
            bool init(hardware_interface::EffortJointInterface* hw,ros::NodeHandle& n);
            void update(const ros::Time& time, const ros::Duration& period);
            void command(const std_msgs::Float64MultiArray::ConstPtr& msg);
            void set_gain(const std_msgs::Float64MultiArray::ConstPtr& msg);
            void starting(const ros::Time& time);
            void stopping(const ros::Time& time);
            void get_Param(ros::NodeHandle& n,std::string name, std::string& param);
        private:
        //joints name
        std::vector<std::string> joints_name_;
        //robot chain
        KDL::Chain robot_chain;
        //solver della dinamica
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
        //M,C,G 
        KDL::JntSpaceInertiaMatrix M_;
        KDL::JntArray G_,C_;
        
        //sub messaggi di comanda
        ros::Subscriber cmd_sub,st_gain_sub;
        //servizio per cambiare guadagni
        KDL::JntArray Kp_,Kv_;
        //variabili di comando
        int new_cmd;
        KDL::JntArray initial_R_state, cmd_R_state, current_cmd, cmd_effort;
        KDL::JntArrayAcc jnt_measured,jnt_des;
        //gravity
        KDL::Vector gravity_;
        
        //Joint handle
        std::vector<hardware_interface::JointHandle> joints_;
        int n_joint;
            
        //trajectory variable
        float lambda;
        int step_;

    };
}