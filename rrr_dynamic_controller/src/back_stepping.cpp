#include <rrr_dynamic_controller/back_stepping.hpp>
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <string>

#define Tf 5.0
#define A3 10
#define A4 -15
#define A5 6
#define PI 3.14159

namespace rrr_dynamic_controller{
    Back_Stepping::Back_Stepping(){}
    Back_Stepping::~Back_Stepping(){}

    void Back_Stepping::check_set_EE_setpoint()
    {
        KDL::Frame check_frame;
        if(fk_solver_->JntToCart(jnt_measured.q,check_frame)<0)
        {
            ROS_ERROR("Somenthing go wrong in fk_solver ");
        }
        //ROS_INFO("%lf,%lf,%lf",pow(check_frame.p(0)- cmd_EE_state(0),2), pow(check_frame.p(1) - cmd_EE_state(1),2) , pow(check_frame.p(2) - cmd_EE_state(2),2));
        if((pow(check_frame.p(0)- cmd_EE_state(0),2) + pow(check_frame.p(1) - cmd_EE_state(1),2) + pow(check_frame.p(2) - cmd_EE_state(2),2)) <0.001)
        {
            
            new_cmd = false;
            time_cmd = 0.0;
            ROS_INFO("Reach End-Effector setpoint");

        }
    }

    bool Back_Stepping::advance_step(double time, KDL::JntArray& advance)
    {
        if(advance.data.size() != 3)
        {
            ROS_ERROR("wrong dimension of advance vector");
            return false;
        }
       //ROS_INFO("time %lf",time);
        if(time > Tf)
            time = Tf;
        advance(0) = (A3/pow(Tf,3))*pow(time,3) + (A4/pow(Tf,4))*pow(time,4)  + (A5/pow(Tf,5))*pow(time,5);
        advance(1) = ((3*A3)/pow(Tf,3))*pow(time,2) + ((4*A4)/pow(Tf,4))*pow(time,3)  + ((5*A5)/pow(Tf,5))*pow(time,4);
        advance(2) = ((6*A3)/pow(Tf,3))*pow(time,1) + ((12*A4)/pow(Tf,4))*pow(time,2)  + ((20*A5)/pow(Tf,5))*pow(time,3);
        // ROS_WARN("il valore di xi e\' [%lf]",advance(0));
        //ROS_INFO("advance [%f,%f,%f],%lf",advance(0),advance(1),advance(2),time);
        return true;
    }
    bool Back_Stepping::compute_xi_(KDL::JntArray advance, KDL::JntArrayAcc& des_traj)
    {
        KDL::JntArrayAcc adv_funct(3);
        if(advance.data.size() != 3)
        {
            ROS_ERROR("wrong dimension of advance vector");
            return false;
        }
        //ROS_INFO("%f,%f,%f",cylinder_cmd(0),cylinder_cmd(1),cylinder_cmd(2));
        for(unsigned int i=0;i<3;i++)
            adv_funct.q(i) = init_cyl(i) + advance(0) * cylinder_cmd(i);
        for(unsigned int i=0;i<3;i++)
            adv_funct.qdot(i) = advance(1) * cylinder_cmd(i);
        for(unsigned int i=0;i<3;i++)
            adv_funct.qdotdot(i) = advance(2) * cylinder_cmd(i);
        //ROS_WARN("il valore di s(t) %f e delle coordinate e\' [%lf,%lf,%lf]",advance(0),adv_funct.q(0),adv_funct.q(1),adv_funct.q(2));
        //compute xi_des
        des_traj.q(0) = adv_funct.q(1) * cos(adv_funct.q(0));
        des_traj.q(1) = adv_funct.q(1) * sin(adv_funct.q(0));
        des_traj.q(2) = adv_funct.q(2);
        //ROS_WARN("il valore di xi e\' [%lf,%lf,%lf]",des_traj.q(0),des_traj.q(1),des_traj.q(2));
        //compute xi_des_dot
        des_traj.qdot(0) = adv_funct.qdot(1)*cos(adv_funct.q(0)) 
        - adv_funct.q(1) * adv_funct.qdot(0) * sin(adv_funct.q(0));
        
        des_traj.qdot(1) = adv_funct.qdot(1)*sin(adv_funct.q(0)) 
        + adv_funct.q(1) * adv_funct.qdot(0) * cos(adv_funct.q(0));
        
        des_traj.qdot(2) = adv_funct.qdot(2);
        //ROS_INFO("vel [%lf,%lf,%lf],%lf",des_traj.qdot(0),des_traj.qdot(1),des_traj.qdot(2),advance(0));
        //compute xi_des_dot_dot
        des_traj.qdotdot(0) = adv_funct.qdotdot(1)*cos(adv_funct.q(0)) 
        - 2*adv_funct.qdot(1)*adv_funct.qdot(0)*sin(adv_funct.q(0))
        - adv_funct.q(1)*adv_funct.qdotdot(0)*sin(adv_funct.q(0)) 
        - adv_funct.q(1)*pow(adv_funct.qdot(0),2)*cos(adv_funct.q(0));
        
        des_traj.qdotdot(1) = adv_funct.qdotdot(1)*sin(adv_funct.q(0)) 
        + 2*adv_funct.qdot(1)*adv_funct.qdot(0)*cos(adv_funct.q(0))
        + adv_funct.q(1)*adv_funct.qdotdot(0)*cos(adv_funct.q(0)) 
        - adv_funct.q(1)*pow(adv_funct.qdot(0),2)*sin(adv_funct.q(0));
        
        des_traj.qdotdot(2) = adv_funct.qdotdot(2);
        //ROS_INFO("acc [%lf,%lf,%lf],%lf",des_traj.qdotdot(0),des_traj.qdotdot(1),des_traj.qdotdot(2),advance(0));
        return true;
    }
    void Back_Stepping::get_Param(ros::NodeHandle& n,std::string name, std::string& param)
    {
        ROS_INFO("param name :%s",name.c_str());
        if(n.getParam(name,param))
                ROS_INFO("exists parameter <robot_description> and its value is %s", param.c_str());
        else
            ROS_ERROR_STREAM("Failed to getParam '" << name << "' (namespace: " << n.getNamespace() << ").");
            
    }
    bool Back_Stepping::init(hardware_interface::EffortJointInterface* hw,ros::NodeHandle& n)
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

        Back_Stepping::get_Param(n,"urdf", urdf);
        Back_Stepping::get_Param(n,"first_link", base_link);
        Back_Stepping::get_Param(n,"ee_link", ee_link);

        //get catena cinematica da urdf

        res = kdl_parser::treeFromFile(urdf,robot_tree);
        if(!res)
            ROS_WARN("Faild Parsing of Robot Tree");
            
        robot_tree.getChain(base_link,ee_link,robot_chain);
        n_joint = robot_chain.getNrOfJoints();
        ROS_INFO("Robot Joints are %d",n_joint);

        id_solver_.reset(new KDL::ChainDynParam(robot_chain, gravity_));
        fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain));
        jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain));
        vel_EE_solver.reset(new KDL::ChainFkSolverVel_recursive(robot_chain));

        //resize all matrix
       // J_.resize(n_joint);
        J_complete.resize(n_joint);
       /* J_inv_.resize(n_joint);
        J_inv_T_.resize(n_joint);
        J_dot_.resize(n_joint);
        old_J_.resize(n_joint);
        J_T_.resize(n_joint);*/

        M_.resize(n_joint);
        C_.resize(n_joint);
        G_.resize(n_joint);

        /*Kv_.resize(n_joint);
        Kp_.resize(n_joint);
*/
        qr_dot_.resize(n_joint);
        qr_dot_dot_.resize(n_joint);
        old_qr_dot.resize(n_joint);
        
        cmd_EE_state.resize(n_joint);
        cmd_effort.resize(n_joint);
        cylinder_cmd.resize(n_joint);
        init_cyl.resize(n_joint);
        advance_s.resize(n_joint);

        jnt_measured.resize(n_joint);
        xi_des.resize(n_joint);
        act_xi.resize(n_joint);
        
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
            &Back_Stepping::command,
            this
        );
        st_gain_sub = n.subscribe(
            "set_gains",
            1,
            &Back_Stepping::set_gain,
            this
        );
        
        ROS_INFO("Compleated initialization of control");
        return true;
    }
   
    void Back_Stepping::update(const ros::Time& time, const ros::Duration& period)
    {
        int res;
        //KDL::JntArray cmd_cyl(3);
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
            //set up error in cylinder coordinate
            if(time_cmd == 0.0)
            {
                res = fk_solver_->JntToCart(jnt_measured.q,initial_EE_state);
                if(res<0)
                {
                    ROS_ERROR("Somenthing go wrong in fk_solver ERROR_NUM:%d",res);
                }
                init_cyl(0) = atan2(initial_EE_state.p(1),initial_EE_state.p(0));
                init_cyl(1) = sqrt(initial_EE_state.p(0)*initial_EE_state.p(0)+
                initial_EE_state.p(1)*initial_EE_state.p(1));
                init_cyl(2) = initial_EE_state.p(2);
                
                for(unsigned int i=0;i<3;i++)
                {
                    cylinder_cmd(i) -= init_cyl(i); 
                }
                
            }
            
            //Compute the time_cmd function 
            //call advanced_step uodate_trajectory
            if(!advance_step(time_cmd,advance_s))
                ROS_ERROR("Error in compitation of s(t) function");
            if(!compute_xi_(advance_s,xi_des))
                ROS_ERROR("Error in compitation of trajectory function");
            
            time_cmd += period.toSec();

            check_set_EE_setpoint();
        }
        //compute inertia matrix
        
        id_solver_->JntToMass(jnt_measured.q,M_);
        id_solver_->JntToCoriolis(jnt_measured.q,jnt_measured.qdot,C_);
        id_solver_->JntToGravity(jnt_measured.q,G_);
        jac_solver_->JntToJac(jnt_measured.q,J_complete);
        //ROS_INFO("dimension of vel jac is %d,%d",J_complete.rows(),J_complete.columns());
        for(unsigned i =0;i<n_joint;i++)
        {
            for(unsigned j=0;j<n_joint;j++)
            {
                //ROS_INFO("J_inv [%d,%d]::%lf",i,j,J_complete.data(i,j));
                J_(i,j)=J_complete.data(i,j);
            }
        }
       
        //ROS_INFO("Computation J_T");
        J_T_= J_.transpose();
        //ROS_INFO("Computation J_inv");
        if(J_.determinant()==0)
            ROS_ERROR("determinatn is null");
        J_inv_ = J_.inverse();
        /*ROS_WARN("J_inv [%lf,%lf,%lf;%lf,%lf,%lf;%lf,%lf,%lf]",
        J_inv_(0,0),J_inv_(0,1),J_inv_(0,2),
        J_inv_(1,0),J_inv_(1,1),J_inv_(1,2),
        J_inv_(2,0),J_inv_(2,1),J_inv_(2,2));
        //ROS_INFO("Computation j_inv_T");*/
        J_inv_T_ = J_inv_.transpose();

        //ROS_INFO("Computation inertia matrix");

        if(fk_solver_->JntToCart(jnt_measured.q,act_EE_pos)<0)
              ROS_ERROR("Somenthing go wrong in fk_solver");
       // KDL::JntArrayVel jnt_vel(3);
        //jnt_vel.q = jnt_measured.q;
        //jnt_vel.qdot = jnt_measured.qdot;

        //vel_EE_solver->JntToCart(jnt_vel,act_EE_vel);
        fk_solver_->JntToCart(jnt_measured.q,act_EE_pos);
       
        //act_twist = act_EE_vel.GetTwist();
        act_EE_vel_.data = J_ * jnt_measured.qdot.data;
         //assing correct value to act_xi
       for(unsigned int i=0;i<3;i++)
        {
            act_xi.q(i) = act_EE_pos.p(i);
           // act_xi.qdot(i) = act_twist(i);
            act_xi.qdot(i) = act_EE_vel_(i);
           
        }
        //ROS_INFO("act_xi [%lf,%lf,%lf] act_xi_dot [%lf,%lf,%lf]",
       // act_xi.q(0),act_xi.q(1),act_xi.q(2),
        //act_xi.qdot(0),act_xi.qdot(1),act_xi.qdot(2));
        

        qr_dot_.data  = J_inv_ *(xi_des.qdot.data  + Kp_*(xi_des.q.data-act_xi.q.data));

        if(time_cmd == 0.0)
        {
            old_qr_dot = qr_dot_;
            //old_J_= J_;
        }
        //ROS_INFO("coputed J_DOT");
     
        qr_dot_dot_.data = (qr_dot_.data - old_qr_dot.data)/period.toSec();
       
        cmd_effort.data = M_.data * qr_dot_dot_.data + C_.data.cwiseProduct(qr_dot_.data) +
        G_.data + Kv_* (qr_dot_.data - jnt_measured.qdot.data) + J_T_ * (xi_des.q.data-act_xi.q.data);

        //J_dot_ = (J_ - old_J_)/period.toSec();
        //prova =  J_T_ * M_.data*J_dot_;
        
       /* ROS_WARN("J_inv [%lf,%lf,%lf;%lf,%lf,%lf;%lf,%lf,%lf]",
        prova(0,0),prova(0,1),prova(0,2),
        prova(1,0),prova(1,1),prova(1,2),
        prova(2,0),prova(2,1),prova(2,2));*/
        //ROS_INFO("joint vel:[%lf,%lf,%lf] ",jnt_measured.qdot(0),jnt_measured.qdot(1),jnt_measured.qdot(2));
        //controller variable
       // KDL::JntArray red_cmd(n_joint);
        //KDL::JntArray cg_comp(n_joint);
       // ROS_INFO("Computation control");
       //ROS_INFO("coputed residual");
        /*for(unsigned int i = 0;i<n_joint;i++)
        {
            red_cmd(i) = xi_des.qdotdot(i) + Kv_(i)*(xi_des.qdot(i) - act_xi.qdot(i)) + Kp_(i)*(xi_des.q(i) - act_xi.q(i));
            
        }*/
       //ROS_INFO("act_pos:[%lf,%lf,%lf] ",act_xi.q(0),act_xi.q(1),act_xi.q(2));
       //ROS_WARN("des_pos:[%lf,%lf,%lf] ",xi_des.q(0),xi_des.q(1),xi_des.q(2));
       // ROS_INFO("red_cmd [%f,%f,%f]",red_cmd.data(0),red_cmd.data(1),red_cmd.data(2));
       // ROS_INFO("coputed feedforward");
        //cg_comp.data = /*J_inv_T_.data*/ (C_.data + G_.data) - J_T_ * M_.data*J_dot_* jnt_measured.qdot.data;
       // ROS_INFO("cg_comp [%f,%f,%f]",cg_comp.data(0),cg_comp.data(1),cg_comp.data(2));
       // cmd_effort.data =/* J_inv_T_.data*/M_.data*J_inv_* red_cmd.data;
        //KDL::Add(cmd_effort,cg_comp,cmd_effort);
        //ROS_WARN("cmd_eff [%f,%f,%f]",cmd_effort.data(0),cmd_effort.data(1),cmd_effort.data(2));
        //set command
        for(unsigned int i=0;i<n_joint;i++)
        {
           joints_[i].setCommand(cmd_effort(i));
        }
          

        


    }
    void Back_Stepping::command(const std_msgs::Float64MultiArray::ConstPtr& msg)
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
                    cmd_EE_state(i) = msg->data[i];
                }
                cylinder_cmd(0) = atan2(cmd_EE_state(1),cmd_EE_state(0));
                cylinder_cmd(1) = sqrt(cmd_EE_state(0)*cmd_EE_state(0)+
                cmd_EE_state(1)*cmd_EE_state(1));
                cylinder_cmd(2) = cmd_EE_state(2);
                new_cmd = true;

            }
        
        
    }
    void Back_Stepping::set_gain(const std_msgs::Float64MultiArray::ConstPtr& msg)
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
    void Back_Stepping::starting(const ros::Time& time)
    {
        int res;
        for(unsigned int i=0;i<n_joint;i++)
        {
            for(unsigned j=0 ;j<n_joint;j++)
            {
                if(i==j)
                {   
                    Kp_(i,j) = 10;
                    Kv_(i,j) = 20;
                }
                else
                {
                    Kp_(i,j) = 0;
                    Kv_(i,j) = 0;
                }
            }
            jnt_measured.q(i) = joints_[i].getPosition();
            jnt_measured.qdot(i) = joints_[i].getVelocity();
            jnt_measured.qdotdot(i) = 0.0;

            res = fk_solver_->JntToCart(jnt_measured.q,initial_EE_state);
            if(res<0)
            {
                ROS_ERROR("Somenthing go wrong in fk_solver ERROR_NUM:%d",res);
            }
            init_cyl(0) = atan2(initial_EE_state.p(1),initial_EE_state.p(0));
            init_cyl(1) = sqrt(initial_EE_state.p(0)*initial_EE_state.p(0)+
            initial_EE_state.p(1)*initial_EE_state.p(1));
            init_cyl(2) = initial_EE_state.p(2);

            cmd_EE_state(0) = 1.0;
            cmd_EE_state(1) = 0.0;
            cmd_EE_state(2) = 1.0;
            cylinder_cmd(0) = 0.0;
            cylinder_cmd(1) = 1.0;
            cylinder_cmd(2) = 1.0;

            
        }
        //da sistemare
       
       
        new_cmd = true;
        time_cmd = 0.0;
        //ROS_INFO("Compleated sterting  of control");
    }
    void Back_Stepping::stopping(const ros::Time& time)
    {}

PLUGINLIB_EXPORT_CLASS(rrr_dynamic_controller::Back_Stepping, controller_interface::ControllerBase);
}

/*
 "layout:
  dim:
  - label: ''
    size: 2
    stride: 0
  data_offset: 0
data:
- 1
- 1
- 1
- 200
- 200
- 200
"


 "layout:
  dim:
  - label: ''
    size: 2
    stride: 0
  data_offset: 0
data:
- 1
- 1
- 1"
*/