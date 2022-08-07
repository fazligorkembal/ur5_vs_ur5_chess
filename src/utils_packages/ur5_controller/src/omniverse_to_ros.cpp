#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<tf2_msgs/TFMessage.h>

class OmniverseToRos
{
    public:
        ros::Subscriber omniverse_to_ros_joint_state_sub;
        ros::Publisher omniverse_to_ros_joint_state_pub;

        ros::Subscriber omniverse_to_ros_tf_sub;
        ros::Publisher omniverse_to_ros_tf_pub;

        sensor_msgs::JointState joint_state_current;
        tf2_msgs::TFMessage tf_current;

        
        std::string ns;
        

    OmniverseToRos(ros::NodeHandle *nh){
        
        
        ROS_INFO("OmniverseToRos initialized");
        
        std::string param;
        ns = ros::this_node::getNamespace();
        ROS_INFO("Starting panda_hw_interface with namespace: %s", ns.c_str());
        
        omniverse_to_ros_joint_state_pub = nh->advertise<sensor_msgs::JointState>(ns + "/joint_states", 1);
        omniverse_to_ros_joint_state_sub = nh->subscribe(ns + "/omniverse_joint_states", 1, &OmniverseToRos::omniverseJointStateCallback, this);
        
        omniverse_to_ros_tf_pub = nh->advertise<tf2_msgs::TFMessage>("/tf", 1);
        omniverse_to_ros_tf_sub = nh->subscribe(ns + "/omniverse_tf", 1, &OmniverseToRos::omniverseTFCallback, this);
        
        
        ros::Rate loop_rate(10);
        while(ros::ok()){
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    void omniverseJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
        joint_state_current = *msg;
        joint_state_current.header.stamp = ros::Time::now();
        omniverse_to_ros_joint_state_pub.publish(joint_state_current);
    }

    void omniverseTFCallback(const tf2_msgs::TFMessage::ConstPtr& msg){
        tf_current = *msg;
        for(int i=0; i < tf_current.transforms.size(); i++){
            tf_current.transforms[i].header.stamp = ros::Time::now();
        }
        omniverse_to_ros_tf_pub.publish(tf_current);
    }

};





int main(int argc, char** argv)
{   
    
    ros::init(argc, argv, "omniverse_to_ros");
    ros::NodeHandle nh;

    OmniverseToRos otr = OmniverseToRos(&nh);

    

}
