#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <Eigen/Core>
#include <Eigen/LU>
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
    class Operational_Space_CT : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        public:
            Operational_Space_CT();
            ~Operational_Space_CT();
            bool init(hardware_interface::EffortJointInterface* hw,ros::NodeHandle& n);
            void update(const ros::Time& time, const ros::Duration& period);
            void command(const std_msgs::Float64MultiArray::ConstPtr& msg);
            void set_gain(const std_msgs::Float64MultiArray::ConstPtr& msg);
            void starting(const ros::Time& time);
            void stopping(const ros::Time& time);
            void get_Param(ros::NodeHandle& n,std::string name, std::string& param);
            bool advance_step(double time,KDL::JntArray& advance);
            bool compute_xi_(KDL::JntArray advance,KDL::JntArrayAcc& des_traj);
            void check_set_EE_setpoint();
        private:
        //joints name
        std::vector<std::string> joints_name_;
        //robot chain
        KDL::Chain robot_chain;
        //solver della dinamica
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> vel_EE_solver;
        //M,C,G 
        Eigen::Matrix3d  J_,J_dot_,old_J_,J_T_,J_inv_,J_inv_T_,prova;
        KDL::Jacobian J_complete;
        
        KDL::JntSpaceInertiaMatrix M_;
        KDL::JntArray G_,C_;
        
        //sub messaggi di comanda
        ros::Subscriber cmd_sub,st_gain_sub;
        //servizio per cambiare guadagni
        KDL::JntArray Kp_,Kv_;
        //variabili di comando
        int new_cmd;
        KDL::JntArray cylinder_cmd, cmd_effort , cmd_EE_state, init_cyl, advance_s, act_EE_vel_;
        KDL::Frame initial_EE_state,act_EE_pos;
        KDL::FrameVel act_EE_vel;
        KDL::Twist act_twist;
        KDL::JntArrayAcc jnt_measured,xi_des,act_xi;
        //gravity
        KDL::Vector gravity_;
        
        //Joint handle
        std::vector<hardware_interface::JointHandle> joints_;
        int n_joint;
            
        //trajectory variable
        
        double time_cmd;

    };
}