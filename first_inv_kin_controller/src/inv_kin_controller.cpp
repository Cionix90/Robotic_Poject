#include <vector>
#include <string>
#include <kdl_parser/kdl_parser.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <boost/scoped_ptr.hpp>
#include <first_inv_kin_controller/pos.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace inv_kin_controller
{
    class InvKinController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
    {
  
    public:
        bool init(hardware_interface::VelocityJointInterface* hw,ros::NodeHandle& n)
        {
            std::string param_name = "joints", base_link,ee_link,s,p1="gazebo";
            KDL::Tree robot_tree; 
            
            int i,res;

            //get parameter from yaml
            //get_type interface
           /* if(!n.getParam(p1, gazebo))
            {
                ROS_ERROR_STREAM("Failed to getParam '" << p1 << "' (namespace: " << n.getNamespace() << ").");
                return false;
            } */
            
            //joints name
            if(!n.getParam(param_name, joints_name_))
            {
                ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
                return false;
            } 
            //urdf file to parse
            if(n.getParam("urdf",s))
                ROS_INFO("exists parameter <robot_description> and its value is %s", s.c_str());
            else
                ROS_ERROR_STREAM("Failed to getParam '" << s << "' (namespace: " << n.getNamespace() << ").");
            
            // base link
            param_name = "first_link";
            if(!n.getParam(param_name, base_link))
            {
                ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
                return false;
            }       

            //tip link
            param_name = "ee_link";
            if(!n.getParam(param_name, ee_link))
            {
                ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
                return false;
            } 

            //parse urdf trea and extract chain
            res = kdl_parser::treeFromFile(s,robot_tree);
            if(!res)
                ROS_WARN("Faild Parsing of Robot Tree");
            
            robot_tree.getChain(base_link,ee_link,robot_chain);
            ROS_INFO("Robot Joints are %d",robot_chain.getNrOfJoints());
            
            // set up solver
            ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(robot_chain));
            fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain));
            fk_vel_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain));

            act_jnt.resize(robot_chain.getNrOfJoints());
            J_.resize(robot_chain.getNrOfJoints());
          /*  act_jnt(0) = 0;
            act_jnt(1) = 0;
            act_jnt(2) = 0;
            J_.resize(robot_chain.getNrOfJoints());

            fk_vel_solver_->JntToJac(act_jnt,J_);
            int jr=J_.rows(),jc=J_.columns();
            ROS_INFO("Jacobian dim is [%d,%d]",jr,jc);
            for(int k=0;k<J_.rows();k++)
            {
                for(int j=0;j<J_.columns();j++)
                ROS_INFO("Jacobian elem [%d,%d] is %f",k,j,J_(k,j));
            }*/
            
            n_joint = joints_name_.size();
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
            cmd_sub = n.subscribe(
                "command",
                5,
                &InvKinController::callback_cmd,
                this
            );
            exec = false;
            return true;
        }
        void update(const ros::Time& time, const ros::Duration& period)
        {
           
            KDL::JntArray cmd_vel;
            Eigen::Matrix3f J_inv,J_pos;
            Eigen::Vector3f x_err,q_dot;
            KDL::Frame act_frame;
            int res;
            cmd_vel.resize(n_joint);
          
            //get joint pos value
            if(exec)
            {
                for(unsigned int i=0;i<n_joint;i++)
                {
                    act_jnt(i) = joints_[i].getPosition();
                    ROS_WARN("value of %d joint is %f",i,act_jnt(i));
                }

                //get Jacobian and compute inverse (it's valid only for 3 joint manipulator)
                fk_vel_solver_->JntToJac(act_jnt,J_);
                
                ROS_INFO("jacobian dim [%d,%d]",J_.rows(),J_.columns());
                //J_inv = J_.data.block(0,0,3,3);
                for(unsigned i =0;i<n_joint;i++)
                {
                    for(unsigned j=0;j<n_joint;j++)
                    {
                        //ROS_INFO("J_inv [%d,%d]::%f",i,j,J_.data(i,j));
                        J_pos(i,j)=J_.data(i,j);
                    }
                }
                if(J_pos.determinant()==0)
                {
                    J_inv = J_pos.transpose();
                    ROS_WARN("Determinant is null, impossible compute inverse");
                }
                else
                    J_inv = J_pos.inverse();

                //

                
                /*for(unsigned i =0;i<n_joint;i++)
                {
                    for(unsigned j=0;j<n_joint;j++)
                    {
                        ROS_WARN("jac_inv%d,%d :::%f",i,j,J_inv(i,j));
                    }
                }*/
                //compute position error
                res = fk_pos_solver_->JntToCart(act_jnt,act_frame);
                if(res<0)
                {
                    ROS_ERROR("Somenthing go wrong in fk_solver ERROR_NUM:%d",res);
                }
                for(unsigned int i = 0;i<n_joint;i++)
                    x_act(i) = act_frame.p(i);
                x_err = x_des - x_act;
                if(x_err.squaredNorm() < 0.00001)
                    exec = false;
                ROS_INFO("pos_act [%f,%f,%f]",x_act(0),x_act(1),x_act(2));
                //ROS_INFO("pos_err [%f,%f,%f]",x_err(0),x_err(1),x_err(2));

                //compute qdot
                q_dot = J_inv * x_err;            
            /*   KDL::Frame act_pos,error_pos;
                
                KDL::Twist error_pos_tw;
            
                fk_pos_solver_->JntToCart(act_jnt,act_pos);
                error_pos = KDL::Frame(act_pos.p - x_des.p);
                error_pos_tw = KDL::Twist(act_pos.p - x_des.p,KDL::Vector(0,0,0));
                ik_vel_solver_->CartToJnt(act_jnt,error_pos_tw,cmd_vel);
    */
                for(unsigned int i=0;i<n_joint;i++)
                {
                    //ROS_INFO("il controllo inviato al %d giunto e' %f",i,cmd_vel(i));
                    joints_[i].setCommand(q_dot(i));
                    //ROS_WARN("value of %d joint command vel is %f",i,q_dot(i));
                }
            }    
        }
        void callback_cmd(first_inv_kin_controller::pos::ConstPtr msg)
        {
            x_des(0) = msg->x_ref;
            x_des(1) = msg->y_ref;
            x_des(2) = msg->z_ref;
            if(!exec)
                exec=true;
        }
        void starting(const ros::Time& time) { }
        void stopping(const ros::Time& time) { }
    private:
        std::vector<hardware_interface::JointHandle> joints_;
        std::vector<std::string> joints_name_;
        int n_joint;
        KDL::Chain robot_chain;
        
        boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> fk_vel_solver_;
        KDL::JntArray act_jnt;
        KDL::Jacobian J_;
        ros::Subscriber cmd_sub;
        Eigen::Vector3f x_des;
        Eigen::Vector3f x_act;
    
        bool exec,gazebo;

        
    

    };
PLUGINLIB_EXPORT_CLASS(inv_kin_controller::InvKinController, controller_interface::ControllerBase);
}