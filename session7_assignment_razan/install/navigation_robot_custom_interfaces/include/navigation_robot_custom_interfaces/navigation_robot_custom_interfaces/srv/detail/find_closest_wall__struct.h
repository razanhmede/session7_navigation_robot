// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from navigation_robot_custom_interfaces:srv/FindClosestWall.idl
// generated code does not contain a copyright notice

#ifndef NAVIGATION_ROBOT_CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__STRUCT_H_
#define NAVIGATION_ROBOT_CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/FindClosestWall in the package navigation_robot_custom_interfaces.
typedef struct navigation_robot_custom_interfaces__srv__FindClosestWall_Request
{
  uint8_t structure_needs_at_least_one_member;
} navigation_robot_custom_interfaces__srv__FindClosestWall_Request;

// Struct for a sequence of navigation_robot_custom_interfaces__srv__FindClosestWall_Request.
typedef struct navigation_robot_custom_interfaces__srv__FindClosestWall_Request__Sequence
{
  navigation_robot_custom_interfaces__srv__FindClosestWall_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} navigation_robot_custom_interfaces__srv__FindClosestWall_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/FindClosestWall in the package navigation_robot_custom_interfaces.
typedef struct navigation_robot_custom_interfaces__srv__FindClosestWall_Response
{
  float x;
  float y;
  float z;
} navigation_robot_custom_interfaces__srv__FindClosestWall_Response;

// Struct for a sequence of navigation_robot_custom_interfaces__srv__FindClosestWall_Response.
typedef struct navigation_robot_custom_interfaces__srv__FindClosestWall_Response__Sequence
{
  navigation_robot_custom_interfaces__srv__FindClosestWall_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} navigation_robot_custom_interfaces__srv__FindClosestWall_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NAVIGATION_ROBOT_CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__STRUCT_H_
