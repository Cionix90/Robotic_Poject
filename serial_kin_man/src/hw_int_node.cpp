#include "../include/serial_kin_man/hw_int_serial.hpp"

/*
int main(int argc,char** argv)
{
    ros::init(argc,argv,"serial_base");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Duration dur;
    Serial_Robot_Interface rob(n);
    controller_manager::ControllerManager cm(&rob);
    rob.init_time();
    sensor_msgs::JointState msg;
    msg.position.resize(3);
    msg.position[1] = 1;
    msg.position[2] = 1;
    msg.position[3] = 1;
    rob.joint_vel_pub.publish(msg);
   /* while(ros::ok())
    {
        dur = rob.get_duration();
        rob.read();
        //cm.update(ros::Time::now(),dur);
        rob.write();

        ros::spinOnce();
        loop_rate.sleep();
    }



    return 0;
}*/


int main(int argc, char **argv)
{

  ros::init(argc, argv, "serial_interface");
  ros::Duration dur;

  ros::NodeHandle n;
  
  
  serial_hardware_interface ser_rob(n);
  controller_manager::ControllerManager cm(&ser_rob);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate loop_rate(10);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  while(ros::ok())
  {
    dur = ser_rob.read();
    cm.update(ser_rob.current_time,dur);
    ser_rob.write();
    loop_rate.sleep();
  }
  

  return 0;
}