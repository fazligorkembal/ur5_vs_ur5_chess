#include <ur5_controller/ur5_hw_interface.h>

namespace ur5_ns
{
    ur5HWInterface::ur5HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model, std::string robot_name):
        ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
    {
        ROS_INFO("Robot Name is: %s", robot_name.c_str());
        ROS_INFO("gRobotHWInterface initialized");
        robot_name = robot_name;
    }    
    
    void ur5HWInterface::init()
    {
        GenericHWInterface::init();
        ROS_INFO("SimHWInterface Ready. Total Joint Count : %ld", joint_names_.size());
        std::string topic_joint_command = nh_.getNamespace() + "/joint_command";
        joint_command_pub = nh_.advertise<sensor_msgs::JointState>(topic_joint_command, 10);
        
        for(int i=0; i < joint_names_.size(); i++){
            ROS_INFO("Joint Name: %s", joint_names_[i].c_str());
        }   
    }

    void ur5HWInterface::read(ros::Duration& elapsed_time)
    {
    }

    void ur5HWInterface::write(ros::Duration& elapsed_time)
    {
        
        sensor_msgs::JointState joint_command;
        joint_command.header.stamp = ros::Time::now();
        joint_command.name = joint_names_;
        joint_command.position = joint_position_command_;
        joint_command.velocity = joint_velocity_command_;
        joint_command.effort = joint_effort_command_;

        
        joint_command.name.push_back("ur5_finger_joint2");
        joint_command.position.push_back(joint_position_command_[6]);
        joint_command.velocity.push_back(joint_velocity_command_[6]);
        joint_command.effort.push_back(joint_effort_command_[6]);
     
        joint_command_pub.publish(joint_command);

        
    }

    void ur5HWInterface::enforceLimits(ros::Duration& period)
    {
        //ROS_INFO("gRobotHWInterface enforceLimits");
    }


}