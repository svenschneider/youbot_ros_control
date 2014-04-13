#ifndef YRC_HARDWARE_INTERFACE_DEVICE_INTERFACE_H
#define YRC_HARDWARE_INTERFACE_DEVICE_INTERFACE_H

#include <vector>
#include <string>

#include <youbot/EthercatMasterInterface.hpp>

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>

namespace yrc_hardware_interface {

class DeviceInterface
{
    public:
        virtual ~DeviceInterface() {}
        virtual bool initialize(const std::string &name, const std::vector<std::string> &joint_names,
                hardware_interface::RobotHW &robot_hw, ros::NodeHandle &nh,
                youbot::EthercatMasterInterface *ethercat_master) = 0;
        virtual bool start() { return true; }
        virtual bool read(const ros::Time &time, const ros::Duration &period) = 0;
        virtual bool write(const ros::Time &time, const ros::Duration &period) = 0;
        virtual bool update() { return true; }
        virtual bool recover() { return true; }
        virtual bool stop() { return true; }
        virtual bool cleanup() { return true; }


    protected:
        template <class T>
        T* getInterface(hardware_interface::RobotHW &robot_hw) {
            // Try to get the joint state interface from the robot hardware interface
            T *interface = robot_hw.get<T>();

            // If the previous step failed, create a new joint state interface and register it with the robot hardware interface
            if (!interface) {
                interface = new T;
                robot_hw.registerInterface(interface);
            }

            return interface;
        }
};

}

#endif
