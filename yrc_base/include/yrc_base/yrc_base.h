#ifndef YRC_BASE_H
#define YRC_BASE_H

#include <youbot/YouBotBase.hpp>

#include <boost/shared_ptr.hpp>

#include <hardware_interface/joint_state_interface.h>

#include <yrc_hardware_interface/device_interface.h>
#include <yrc_hardware_interface/omni_base_state_interface.h>
#include <yrc_hardware_interface/omni_base_command_interface.h>

namespace yrc_base {

class BaseDeviceInterface : public yrc_hardware_interface::DeviceInterface
{
    public:
        BaseDeviceInterface();
        virtual ~BaseDeviceInterface();
        bool initialize(const std::string &name, const std::vector<std::string> &joint_names,
                hardware_interface::RobotHW &robot_hw, ros::NodeHandle &nh,
                youbot::EthercatMasterInterface *ethercat_master);
        bool start();
        bool read(const ros::Time &time, const ros::Duration &period);
        bool write(const ros::Time &time, const ros::Duration &period);


    private:
        youbot::EthercatMasterInterface *ethercat_master_;

        bool is_initialized_;
        std::string configuration_path_;
        unsigned int joint_count_;
        std::string base_name_;
        boost::shared_ptr<youbot::YouBotBase> base_;

        hardware_interface::JointStateInterface *joint_state_if_;
        std::vector<double> joint_positions_;
        std::vector<double> joint_velocities_;
        std::vector<double> joint_efforts_;

        yrc_hardware_interface::OmniBaseStateInterface *base_state_if_;
        double base_state_x_;
        double base_state_y_;
        double base_state_theta_;

        yrc_hardware_interface::OmniBaseCommandInterface *base_command_if_;
        double base_cmd_x_;
        double base_cmd_y_;
        double base_cmd_theta_;
};

}

#endif
