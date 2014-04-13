#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <hardware_interface/robot_hw.h>
#include <yrc_hardware_interface/device_interface.h>

#include <youbot/EthercatMasterInterface.hpp>

/*
extern "C"{
#include <ethercattype.h>
#include <ethercatmain.h>
}
*/

class YouBot : public hardware_interface::RobotHW
{
    public:
        YouBot() {}
        virtual ~YouBot() {}
};

class MockEthercatMasterInterface : public youbot::EthercatMasterInterface
{
    public:
        bool isThreadActive() { return false; }

        unsigned int getNumberOfSlaves() const { return 0; }

        void AutomaticSendOn(const bool enableAutomaticSend) {}

        void AutomaticReceiveOn(const bool enableAutomaticReceive) {}

        void getEthercatDiagnosticInformation(std::vector<ec_slavet>& ethercatSlaveInfos) {}

        bool sendProcessData() { return true; }

        bool receiveProcessData() { return true; }

        bool isErrorInSoemDriver() { return false; }

        bool isEtherCATConnectionEstablished() { return true; }

        void registerJointLimitMonitor(youbot::JointLimitMonitor* object, const unsigned int JointNumber) {}


    private:
        void setMsgBuffer(const youbot::YouBotSlaveMsg& msgBuffer, const unsigned int jointNumber) {}

        void getMsgBuffer(const unsigned int jointNumber, youbot::YouBotSlaveMsg& returnMsg) {}

        void setMailboxMsgBuffer(const youbot::YouBotSlaveMailboxMsg& msgBuffer, const unsigned int jointNumber) {}

        bool getMailboxMsgBuffer(youbot::YouBotSlaveMailboxMsg& mailboxMsg, const unsigned int jointNumber) { return true; }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "oodl_ros_control");
    ros::NodeHandle nh("~");

    std::string config_path;
    nh.getParam("configuration_path", config_path);

    YouBot youbot;
    youbot::EthercatMasterInterface *ethercat_master = new MockEthercatMasterInterface;

    pluginlib::ClassLoader<yrc_hardware_interface::DeviceInterface> base_loader("yrc_hardware_interface", "yrc_hardware_interface::DeviceInterface");

    try {
        boost::shared_ptr<yrc_hardware_interface::DeviceInterface> base = base_loader.createInstance("yrc_base::BaseDeviceInterface");

        /*
         *  numbering of youBot wheels:
         *
         *    FRONT
         *
         * 1 ---+--- 2
         *      |
         *      |
         *      |
         *      |
         * 3 ---+--- 4
         *
         *    BACK
         */
        std::vector<std::string> joint_names;
        joint_names.push_back("wheel_joint_fl");    // Wheel #1
        joint_names.push_back("wheel_joint_fr");    // Wheel #2
        joint_names.push_back("wheel_joint_bl");    // Wheel #3
        joint_names.push_back("wheel_joint_br");    // Wheel #4

        base->initialize("base", joint_names, youbot, nh, ethercat_master);
    } catch (pluginlib::PluginlibException &e) {
        ROS_ERROR("The plugin failed to load. Error: %s", e.what());
    }

    return 0;
}
