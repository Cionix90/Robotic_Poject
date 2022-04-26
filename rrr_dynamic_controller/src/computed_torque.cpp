#include <rrr_dynamic_controller/computed_torque.hpp>
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <string>

#define PI 3.14159

namespace rrr_dynamic_controller{
    Computed_Torque_controller::Computed_Torque_controller(){}
    Computed_Torque_controller::~Computed_Torque_controller(){}

    void Computed_Torque_controller::get_Param(ros::NodeHandle& n,std::string name, std::string& param)
    {
        ROS_INFO("param name :%s",name.c_str());
        if(n.getParam(name,param))
                ROS_INFO("exists parameter <robot_description> and its value is %s", param.c_str());
        else
            ROS_ERROR_STREAM("Failed to getParam '" << name << "' (namespace: " << n.getNamespace() << ").");
            
    }
    bool Computed_Torque_controller::init(hardware_interface::EffortJointInterface* hw,ros::NodeHandle& n)
    {
        std::string ee_link,base_link,urdf;
        int res,i;
        KDL::Tree robot_tree;

        gravity_ = KDL::Vector::Zero();
        gravity_(2) = -9.81;
        new_cmd = false;
        ROS_INFO("Computed Torque make init");
        //get param
        if(n.getParam("joints",joints_name_))
                ROS_INFO("exists parameter <robot_description> and its value is joints");
        else
            ROS_ERROR_STREAM("Failed to getParam joints (namespace: " << n.getNamespace() << ").");

        Computed_Torque_controller::get_Param(n,"urdf", urdf);
        Computed_Torque_controller::get_Param(n,"first_link", base_link);
        Computed_Torque_controller::get_Param(n,"ee_link", ee_link);

        //get catena cinematica da urdf

        res = kdl_parser::treeFromFile(urdf,robot_tree);
        if(!res)
            ROS_WARN("Faild Parsing of Robot Tree");
            
        robot_tree.getChain(base_link,ee_link,robot_chain);
        n_joint = robot_chain.getNrOfJoints();
        ROS_INFO("Robot Joints are %d",n_joint);

        id_solver_.reset(new KDL::ChainDynParam(robot_chain, gravity_));

        //resize all matrix
        M_.resize(n_joint);
        C_.resize(n_joint);
        G_.resize(n_joint);
        Kv_.resize(n_joint);
        Kp_.resize(n_joint);
        initial_R_state.resize(n_joint);
        cmd_effort.resize(n_joint);
        cmd_R_state.resize(n_joint);
        current_cmd.resize(n_joint);

        jnt_measured.resize(n_joint);
        jnt_des.resize(n_joint);
        
        //set JointHandle
        for(i=0;i<n_joint;i++)
        {
            try
            {
            ROS_INFO("the %d jointHandle is called %s",i,joints_name_[i].c_str());
            joints_.push_back(hw->getHandle(joints_name_[i]));
            }
            catch(const hardware_interface::HardwareInterfaceException& e)
            {
            ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;     
            }
        }
        //set subribers
        cmd_sub = n.subscribe(
            "command",
            1,
            &Computed_Torque_controller::command,
            this
        );
        st_gain_sub = n.subscribe(
            "set_gains",
            1,
            &Computed_Torque_controller::set_gain,
            this
        );
        
        ROS_INFO("Compleated initialization of control");
        return true;
    }
   
    void Computed_Torque_controller::update(const ros::Time& time, const ros::Duration& period)
    {
        //get measures
         for(unsigned int i=0;i<n_joint;i++)
        {
            jnt_measured.q(i) = joints_[i].getPosition();
            jnt_measured.qdot(i) = joints_[i].getVelocity();
            jnt_measured.qdotdot(i) = 0.0;
        }
        //available new joint pos setpoint
        if(new_cmd)
        {
            //is the first itereation of new command
            if(step_ == 0)
            {
                initial_R_state = jnt_measured.q;
            }
            //trajecory between 2 setpoint is defined by tanh function 
            double th = tanh(PI - lambda*step_);
            double csh = cosh(PI - lambda*step_);
            double sh2 = 1/(csh*csh);
            
            //compute next step
            for(unsigned int i = 0; i<n_joint ;i++)
            {
                current_cmd(i) = cmd_R_state(i) - initial_R_state(i);
                jnt_des.q(i) = current_cmd(i)*0.5*(1-th) + initial_R_state(i);
                jnt_des.qdot(i) = current_cmd(i)*0.5*lambda*sh2;
                jnt_des.qdotdot(i) = current_cmd(i)*lambda*lambda*sh2*th;
            }
            step_++;
            if(jnt_des.q == cmd_R_state)
            {
                new_cmd = false;
                step_ = 0;
                ROS_INFO("Reach setpoint");
            }
        }
        //compute inertia matrix
       // ROS_INFO("Computation inertia matrix");
        id_solver_->JntToMass(jnt_measured.q,M_);
        id_solver_->JntToCoriolis(jnt_measured.q,jnt_measured.qdot,C_);
        id_solver_->JntToGravity(jnt_measured.q,G_);

        //controller variable
        KDL::JntArray red_cmd(n_joint);
        KDL::JntArray cg_comp(n_joint);
       // ROS_INFO("Computation control");
        for(unsigned int i = 0;i<n_joint;i++)
        {
            red_cmd(i) = jnt_des.qdotdot(i) + Kv_(i)*(jnt_des.qdot(i) - jnt_measured.qdot(i)) + Kp_(i)*(jnt_des.q(i) - jnt_measured.q(i));
            cg_comp(i) = G_(i) + C_(i);
        }
        //ROS_INFO("Computation pid control per M_");
        cmd_effort.data = M_.data*red_cmd.data;
        KDL::Add(cmd_effort,cg_comp,cmd_effort);

        //set command
        for(unsigned int i=0;i<n_joint;i++)
        {
           joints_[i].setCommand(cmd_effort(i));
        }
            

        


    }
    void Computed_Torque_controller::command(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        if(msg->data.size()!= n_joint)
            ROS_ERROR("Joint position number has wrong number of elements,it's %u but they should be %d",
            (unsigned int) msg->data.size(),n_joint);
        else
            if(new_cmd)
                ROS_WARN("there is just an other command to be executed");
            else
            {
                for(unsigned int i=0;i<n_joint;i++)
                {
                    cmd_R_state(i) = msg->data[i];
                }
                new_cmd = true;
            }
        
        
    }
    void Computed_Torque_controller::set_gain(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
         if(msg->data.size()!= n_joint*2)
            ROS_ERROR("Joint position number has wrong number of elements,it's %u but they should be %d",
            (unsigned int) msg->data.size(),2*n_joint);
        else
            if(new_cmd)
                ROS_WARN("there is just an other command to be executed; Gain shoudn't be change");
            else
            {
                for(unsigned int i=0;i<n_joint;i++)
                {
                   Kp_(i) = msg->data[i];
                   Kv_(i) = msg->data[i + n_joint]; 
                }
            }
    }
    void Computed_Torque_controller::starting(const ros::Time& time)
    {
        for(unsigned int i=0;i<n_joint;i++)
        {
            Kp_(i) = 0.5;
            Kv_(i) = 1;
            jnt_measured.q(i) = joints_[i].getPosition();
            jnt_measured.qdot(i) = joints_[i].getVelocity();
            jnt_measured.qdotdot(i) = 0.0;
            jnt_des.q(i) = jnt_measured.q(i);
        }
        lambda =  0.1;
        new_cmd = false;
        step_ = 0;
        //ROS_INFO("Compleated sterting  of control");
    }
    void Computed_Torque_controller::stopping(const ros::Time& time)
    {}

PLUGINLIB_EXPORT_CLASS(rrr_dynamic_controller::Computed_Torque_controller, controller_interface::ControllerBase);
}

/*
 "layout:
  dim:
  - label: ''
    size: 5
    stride: 0
  data_offset: 0
data:
- 100
- 100
- 100
- 200
- 200
- 200
"
*/