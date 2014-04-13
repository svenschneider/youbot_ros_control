#include <yrc_base/yrc_base.h>

#include <boost/units/systems/si.hpp>

#include <pluginlib/class_list_macros.h>

using namespace yrc_base;

// Declare the upper BaseDeviceInterface as a plugin of the generic DeviceInterface
PLUGINLIB_EXPORT_CLASS(yrc_base::BaseDeviceInterface, yrc_hardware_interface::DeviceInterface)


BaseDeviceInterface::BaseDeviceInterface()
{
    ethercat_master_ = 0;
    joint_count_ = 0;

    joint_state_if_ = 0;
    base_state_if_ = 0;
    base_command_if_ = 0;

    is_initialized_ = false;

    base_state_x_ = 0.0;
    base_state_y_ = 0.0;
    base_state_theta_ = 0.0;

    base_cmd_x_ = 0.0;
    base_cmd_y_ = 0.0;
    base_cmd_theta_ = 0.0;
}


BaseDeviceInterface::~BaseDeviceInterface()
{
    // TODO: Who cleans up the allocated memory???
}


bool BaseDeviceInterface::initialize(const std::string &name, const std::vector<std::string> &joint_names,
        hardware_interface::RobotHW &robot_hw, ros::NodeHandle &nh, youbot::EthercatMasterInterface *ethercat_master)
{
    base_name_ = name;
    joint_count_ = joint_names.size();

    // Validate that exactly four joints are provided
    if (joint_count_ != 4) {
        ROS_ERROR("Expected 4 joints, but got %i joints.", joint_count_);
        return false;
    }

    // Initialize the interfaces of the base
    joint_state_if_ = getInterface<hardware_interface::JointStateInterface>(robot_hw);
    base_state_if_ = getInterface<yrc_hardware_interface::OmniBaseStateInterface>(robot_hw);
    base_command_if_ = getInterface<yrc_hardware_interface::OmniBaseCommandInterface>(robot_hw);

    // Initialize the joint state vectors
    joint_positions_.resize(joint_count_, 0.0);
    joint_velocities_.resize(joint_count_, 0.0);
    joint_efforts_.resize(joint_count_, 0.0);

    // Register each joint with the state interface
    for (unsigned int i = 0; i < joint_count_; i++) {
        hardware_interface::JointStateHandle handle(joint_names[i],
                &joint_positions_[i],
                &joint_velocities_[i],
                &joint_efforts_[i]);
        joint_state_if_->registerHandle(handle);
    }


    // Get the OODL configuration path
    if (!nh.hasParam("configuration_path")) {
        ROS_ERROR("No OODL configuration path provided.");
        return false;
    }
    nh.getParam("configuration_path", configuration_path_);

    // Validate and save the interface to the EtherCAT master
    if (!ethercat_master) {
        ROS_ERROR("EtherCAT master interface is NULL.");
        return false;
    }
    ethercat_master_ = ethercat_master;


    is_initialized_ = true;

    return false;
}


bool BaseDeviceInterface::start()
{
    if (!is_initialized_) return false;

    try {
        base_.reset(new youbot::YouBotBase(base_name_, configuration_path_));
        base_->doJointCommutation();
    } catch (std::exception &e) {
        ROS_FATAL("Base could not be initialized. Error: %s", e.what());
        return false;
    }

    return false;
}


bool BaseDeviceInterface::read(const ros::Time &time, const ros::Duration &period)
{
    if (!is_initialized_) return false;

    try {
        // Ensure, that all joint values will be received at the same time
        ethercat_master_->AutomaticReceiveOn(false);

        // Read the joint values from the base
        youbot::JointSensedAngle current_angle;
        youbot::JointSensedVelocity current_velocity;
        youbot::JointSensedTorque current_torque;

        for (unsigned int i = 0; i < joint_count_; i++)
        {
            // youBot joints start with 1, not with 0 -> i + 1
            base_->getBaseJoint(i + 1).getData(current_angle);
            base_->getBaseJoint(i + 1).getData(current_velocity);
            base_->getBaseJoint(i + 1).getData(current_torque);

            joint_positions_[i] = current_angle.angle.value();
            joint_velocities_[i] = current_velocity.angularVelocity.value();
            joint_efforts_[i] = current_torque.torque.value();
        }


        // Read the x/y/theta velocity interface from the base
        boost::units::quantity<boost::units::si::velocity> longitudinal_velocity;
        boost::units::quantity<boost::units::si::velocity> transversal_velocity;
        boost::units::quantity<boost::units::si::angular_velocity> angular_velocity;

        base_->getBaseVelocity(longitudinal_velocity, transversal_velocity, angular_velocity);
        base_state_x_ = longitudinal_velocity.value();
        base_state_y_ = transversal_velocity.value();
        base_state_theta_ = angular_velocity.value();

        // Ensure, that all joint values will be received at the same time
        ethercat_master_->AutomaticReceiveOn(true);
    } catch (youbot::EtherCATConnectionException &e) {
        is_initialized_ = false;
        ROS_ERROR("Lost EtherCAT connection. Error: %s", e.what());

        return false;
    }

    return false;
}


bool BaseDeviceInterface::write(const ros::Time &time, const ros::Duration &period)
{
    if (!is_initialized_) return false;

    boost::units::quantity<boost::units::si::velocity> longitudinal_velocity;
    boost::units::quantity<boost::units::si::velocity> transversal_velocity;
    boost::units::quantity<boost::units::si::angular_velocity> angular_velocity;

    /*
     * Frame in OODL:
     *
     *       FRONT
     *
     *         X
     *         ^
     *         |
     *         |
     *         |
     * Y <-----+
     *
     *        BACK
     *
     * Positive angular velocity means turning counterclockwise
     *
     */

    longitudinal_velocity = base_cmd_x_ * boost::units::si::meter_per_second;
    transversal_velocity = base_cmd_y_ * boost::units::si::meter_per_second;
    angular_velocity = base_cmd_theta_ * boost::units::si::radian_per_second;

    try {
        base_->setBaseVelocity(longitudinal_velocity, transversal_velocity, angular_velocity);
    } catch (std::exception &e) {
        ROS_WARN("Cannot set base velocities: %s", e.what());
    }

    return false;
}
