#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <sensor_msgs/JointState.h>
#include <kinematic_simulator/joint_vel.h>
// simple class which implement the Kinematic symulation of a manipulator 

class kin_sim_class
{
private:
    KDL::JntArray joint_;
    KDL::JntArray joint_vel_;
    sensor_msgs::JointState jointState;
    ros::Time internal_time;
    ros::Publisher pub_jnt;
    ros::Publisher pub_control_jnt;
    ros::Subscriber sub_jnt_vel;
    ros::ServiceServer joints_srv;
    ros::Time old_time;
public:
    int jnt_num;
    kin_sim_class(ros::NodeHandle &node);
    int get_joint_size(bool deb);
    bool init(KDL::JntArray init_pos);
    void publish_joint();
    void subcribe_joint_vel(const kinematic_simulator::joint_vel::ConstPtr &msg,int jnt_id);
    void set_joint_vel(int index, double value);
    void callback_update_vel(const sensor_msgs::JointState::ConstPtr &msg);
    void update_q();
    ~kin_sim_class() {};

};
