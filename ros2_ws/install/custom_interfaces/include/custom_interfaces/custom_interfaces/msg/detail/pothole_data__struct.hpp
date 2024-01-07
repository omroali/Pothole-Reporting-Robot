// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/PotholeData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__POTHOLE_DATA__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__POTHOLE_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pothole_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__msg__PotholeData __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__PotholeData __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PotholeData_
{
  using Type = PotholeData_<ContainerAllocator>;

  explicit PotholeData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pothole_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->radius = 0ll;
    }
  }

  explicit PotholeData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    pothole_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->radius = 0ll;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _pothole_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pothole_pose_type pothole_pose;
  using _radius_type =
    int64_t;
  _radius_type radius;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__pothole_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pothole_pose = _arg;
    return *this;
  }
  Type & set__radius(
    const int64_t & _arg)
  {
    this->radius = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::msg::PotholeData_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::PotholeData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::PotholeData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::PotholeData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::PotholeData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::PotholeData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::PotholeData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::PotholeData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::PotholeData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::PotholeData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__PotholeData
    std::shared_ptr<custom_interfaces::msg::PotholeData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__PotholeData
    std::shared_ptr<custom_interfaces::msg::PotholeData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PotholeData_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->pothole_pose != other.pothole_pose) {
      return false;
    }
    if (this->radius != other.radius) {
      return false;
    }
    return true;
  }
  bool operator!=(const PotholeData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PotholeData_

// alias to use template instance with default allocator
using PotholeData =
  custom_interfaces::msg::PotholeData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__POTHOLE_DATA__STRUCT_HPP_
