#ifndef UR5_HW_INTERFACE_H
#define UR5_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

namespace ur5_ns
{
    class ur5HWInterface : public ros_control_boilerplate::GenericHWInterface
    {
        public:
            /**
             * \brief Constructor
             * \param nh - Node handle for topics.
             */
            ur5HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL, std::string robot_name = "");

            virtual void init();

            virtual void read(ros::Duration& elapsed_time);

            virtual void write(ros::Duration& elapsed_time);

            virtual void enforceLimits(ros::Duration& period);
        
        protected:
            sensor_msgs::JointState joint_state_current;
            ros::Publisher joint_command_pub;
            std::string robot_name;
    };
}

#endif