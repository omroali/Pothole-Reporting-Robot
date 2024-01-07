// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/PotholeData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__POTHOLE_DATA__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__POTHOLE_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'pothole_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/PotholeData in the package custom_interfaces.
typedef struct custom_interfaces__msg__PotholeData
{
  rosidl_runtime_c__String name;
  geometry_msgs__msg__Pose pothole_pose;
  int64_t radius;
} custom_interfaces__msg__PotholeData;

// Struct for a sequence of custom_interfaces__msg__PotholeData.
typedef struct custom_interfaces__msg__PotholeData__Sequence
{
  custom_interfaces__msg__PotholeData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__PotholeData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__POTHOLE_DATA__STRUCT_H_
