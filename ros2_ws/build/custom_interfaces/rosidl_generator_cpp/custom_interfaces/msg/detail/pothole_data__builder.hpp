// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/PotholeData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__POTHOLE_DATA__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__POTHOLE_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/pothole_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_PotholeData_radius
{
public:
  explicit Init_PotholeData_radius(::custom_interfaces::msg::PotholeData & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::PotholeData radius(::custom_interfaces::msg::PotholeData::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::PotholeData msg_;
};

class Init_PotholeData_pothole_pose
{
public:
  explicit Init_PotholeData_pothole_pose(::custom_interfaces::msg::PotholeData & msg)
  : msg_(msg)
  {}
  Init_PotholeData_radius pothole_pose(::custom_interfaces::msg::PotholeData::_pothole_pose_type arg)
  {
    msg_.pothole_pose = std::move(arg);
    return Init_PotholeData_radius(msg_);
  }

private:
  ::custom_interfaces::msg::PotholeData msg_;
};

class Init_PotholeData_name
{
public:
  Init_PotholeData_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PotholeData_pothole_pose name(::custom_interfaces::msg::PotholeData::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_PotholeData_pothole_pose(msg_);
  }

private:
  ::custom_interfaces::msg::PotholeData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::PotholeData>()
{
  return custom_interfaces::msg::builder::Init_PotholeData_name();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__POTHOLE_DATA__BUILDER_HPP_
