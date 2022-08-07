#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <ur5_controller/ur5_hw_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_hw_interface");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();

    std::shared_ptr<ur5_ns::ur5HWInterface> ur5_hw_interface(new ur5_ns::ur5HWInterface(nh));
    ur5_hw_interface->init();

    ros_control_boilerplate::GenericHWControlLoop control_loop(nh, ur5_hw_interface);
    control_loop.run();  // Blocks until shutdown signal recieved

    return 0;
}
