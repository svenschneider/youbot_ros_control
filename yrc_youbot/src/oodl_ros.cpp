
// see: https://github.com/Igorec/ros_control/tree/composite_hardware_interface
virtual bool configure()
{
  ros::NodeHandle node_("~");

  // Read device list
  XmlRpc::XmlRpcValue device_list;
  if (!node_.getParam("device_list", device_list)) {
    ROS_ERROR("Could not load %s/device_list parameter.", node_.getNamespace().c_str());
    return false;
  }

  if (device_list.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("Device list should be a map with a name as key.");
    return false;
  }

  // Load and configure each device
  loader_.reset(new DeviceLoader<DeviceHW>("composite_hardware_interface", "composite_hardware_interface::DeviceHW"));

  for (XmlRpc::XmlRpcValue::iterator it = device_list.begin(); it != device_list.end(); it++) {
    DeviceHwPtr dev;
    std::string dev_name = it->first;
    ros::NodeHandle dev_node(node_, "device_list/" + dev_name);

    std::string dev_type;
    if (!dev_node.getParam("type", dev_type)) {
      ROS_ERROR("Could not find a type attribute for the '%s' hardware interface.", dev_name.c_str());
      return false;
    }

    try {
      dev = loader_->createInstance(it->second["type"]);
    } catch (const std::exception &e) {
      ROS_ERROR("Could not load class %s: %s.", dev_type.c_str(), e.what());
      return false;
    }

    if (!dev->configure(interfaces_, resources_, urdf_model_, dev_name, dev_node)) {
      ROS_ERROR("Could not configure '%s' hardware interface.", dev_name.c_str());
      return false;
    }

    devices_[dev_name] = dev;
  }

  configured_ = true;
  return true;
}
