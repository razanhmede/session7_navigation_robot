// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from navigation_robot_custom_interfaces:action/MeasureLapTime.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__struct.h"
#include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MeasureLapTime_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MeasureLapTime_Goal_type_support_ids_t;

static const _MeasureLapTime_Goal_type_support_ids_t _MeasureLapTime_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MeasureLapTime_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MeasureLapTime_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MeasureLapTime_Goal_type_support_symbol_names_t _MeasureLapTime_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, navigation_robot_custom_interfaces, action, MeasureLapTime_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_robot_custom_interfaces, action, MeasureLapTime_Goal)),
  }
};

typedef struct _MeasureLapTime_Goal_type_support_data_t
{
  void * data[2];
} _MeasureLapTime_Goal_type_support_data_t;

static _MeasureLapTime_Goal_type_support_data_t _MeasureLapTime_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MeasureLapTime_Goal_message_typesupport_map = {
  2,
  "navigation_robot_custom_interfaces",
  &_MeasureLapTime_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_MeasureLapTime_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_MeasureLapTime_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MeasureLapTime_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MeasureLapTime_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace navigation_robot_custom_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_Goal)() {
  return &::navigation_robot_custom_interfaces::action::rosidl_typesupport_c::MeasureLapTime_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MeasureLapTime_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MeasureLapTime_Result_type_support_ids_t;

static const _MeasureLapTime_Result_type_support_ids_t _MeasureLapTime_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MeasureLapTime_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MeasureLapTime_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MeasureLapTime_Result_type_support_symbol_names_t _MeasureLapTime_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, navigation_robot_custom_interfaces, action, MeasureLapTime_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_robot_custom_interfaces, action, MeasureLapTime_Result)),
  }
};

typedef struct _MeasureLapTime_Result_type_support_data_t
{
  void * data[2];
} _MeasureLapTime_Result_type_support_data_t;

static _MeasureLapTime_Result_type_support_data_t _MeasureLapTime_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MeasureLapTime_Result_message_typesupport_map = {
  2,
  "navigation_robot_custom_interfaces",
  &_MeasureLapTime_Result_message_typesupport_ids.typesupport_identifier[0],
  &_MeasureLapTime_Result_message_typesupport_symbol_names.symbol_name[0],
  &_MeasureLapTime_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MeasureLapTime_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MeasureLapTime_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace navigation_robot_custom_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_Result)() {
  return &::navigation_robot_custom_interfaces::action::rosidl_typesupport_c::MeasureLapTime_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MeasureLapTime_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MeasureLapTime_Feedback_type_support_ids_t;

static const _MeasureLapTime_Feedback_type_support_ids_t _MeasureLapTime_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MeasureLapTime_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MeasureLapTime_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MeasureLapTime_Feedback_type_support_symbol_names_t _MeasureLapTime_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, navigation_robot_custom_interfaces, action, MeasureLapTime_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_robot_custom_interfaces, action, MeasureLapTime_Feedback)),
  }
};

typedef struct _MeasureLapTime_Feedback_type_support_data_t
{
  void * data[2];
} _MeasureLapTime_Feedback_type_support_data_t;

static _MeasureLapTime_Feedback_type_support_data_t _MeasureLapTime_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MeasureLapTime_Feedback_message_typesupport_map = {
  2,
  "navigation_robot_custom_interfaces",
  &_MeasureLapTime_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_MeasureLapTime_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_MeasureLapTime_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MeasureLapTime_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MeasureLapTime_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace navigation_robot_custom_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_Feedback)() {
  return &::navigation_robot_custom_interfaces::action::rosidl_typesupport_c::MeasureLapTime_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MeasureLapTime_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MeasureLapTime_SendGoal_Request_type_support_ids_t;

static const _MeasureLapTime_SendGoal_Request_type_support_ids_t _MeasureLapTime_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MeasureLapTime_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MeasureLapTime_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MeasureLapTime_SendGoal_Request_type_support_symbol_names_t _MeasureLapTime_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, navigation_robot_custom_interfaces, action, MeasureLapTime_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_robot_custom_interfaces, action, MeasureLapTime_SendGoal_Request)),
  }
};

typedef struct _MeasureLapTime_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _MeasureLapTime_SendGoal_Request_type_support_data_t;

static _MeasureLapTime_SendGoal_Request_type_support_data_t _MeasureLapTime_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MeasureLapTime_SendGoal_Request_message_typesupport_map = {
  2,
  "navigation_robot_custom_interfaces",
  &_MeasureLapTime_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_MeasureLapTime_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_MeasureLapTime_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MeasureLapTime_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MeasureLapTime_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace navigation_robot_custom_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_SendGoal_Request)() {
  return &::navigation_robot_custom_interfaces::action::rosidl_typesupport_c::MeasureLapTime_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MeasureLapTime_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MeasureLapTime_SendGoal_Response_type_support_ids_t;

static const _MeasureLapTime_SendGoal_Response_type_support_ids_t _MeasureLapTime_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MeasureLapTime_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MeasureLapTime_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MeasureLapTime_SendGoal_Response_type_support_symbol_names_t _MeasureLapTime_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, navigation_robot_custom_interfaces, action, MeasureLapTime_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_robot_custom_interfaces, action, MeasureLapTime_SendGoal_Response)),
  }
};

typedef struct _MeasureLapTime_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _MeasureLapTime_SendGoal_Response_type_support_data_t;

static _MeasureLapTime_SendGoal_Response_type_support_data_t _MeasureLapTime_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MeasureLapTime_SendGoal_Response_message_typesupport_map = {
  2,
  "navigation_robot_custom_interfaces",
  &_MeasureLapTime_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_MeasureLapTime_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_MeasureLapTime_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MeasureLapTime_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MeasureLapTime_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace navigation_robot_custom_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_SendGoal_Response)() {
  return &::navigation_robot_custom_interfaces::action::rosidl_typesupport_c::MeasureLapTime_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MeasureLapTime_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MeasureLapTime_SendGoal_type_support_ids_t;

static const _MeasureLapTime_SendGoal_type_support_ids_t _MeasureLapTime_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MeasureLapTime_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MeasureLapTime_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MeasureLapTime_SendGoal_type_support_symbol_names_t _MeasureLapTime_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, navigation_robot_custom_interfaces, action, MeasureLapTime_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_robot_custom_interfaces, action, MeasureLapTime_SendGoal)),
  }
};

typedef struct _MeasureLapTime_SendGoal_type_support_data_t
{
  void * data[2];
} _MeasureLapTime_SendGoal_type_support_data_t;

static _MeasureLapTime_SendGoal_type_support_data_t _MeasureLapTime_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MeasureLapTime_SendGoal_service_typesupport_map = {
  2,
  "navigation_robot_custom_interfaces",
  &_MeasureLapTime_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_MeasureLapTime_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_MeasureLapTime_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t MeasureLapTime_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MeasureLapTime_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace navigation_robot_custom_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_SendGoal)() {
  return &::navigation_robot_custom_interfaces::action::rosidl_typesupport_c::MeasureLapTime_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MeasureLapTime_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MeasureLapTime_GetResult_Request_type_support_ids_t;

static const _MeasureLapTime_GetResult_Request_type_support_ids_t _MeasureLapTime_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MeasureLapTime_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MeasureLapTime_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MeasureLapTime_GetResult_Request_type_support_symbol_names_t _MeasureLapTime_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, navigation_robot_custom_interfaces, action, MeasureLapTime_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_robot_custom_interfaces, action, MeasureLapTime_GetResult_Request)),
  }
};

typedef struct _MeasureLapTime_GetResult_Request_type_support_data_t
{
  void * data[2];
} _MeasureLapTime_GetResult_Request_type_support_data_t;

static _MeasureLapTime_GetResult_Request_type_support_data_t _MeasureLapTime_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MeasureLapTime_GetResult_Request_message_typesupport_map = {
  2,
  "navigation_robot_custom_interfaces",
  &_MeasureLapTime_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_MeasureLapTime_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_MeasureLapTime_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MeasureLapTime_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MeasureLapTime_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace navigation_robot_custom_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_GetResult_Request)() {
  return &::navigation_robot_custom_interfaces::action::rosidl_typesupport_c::MeasureLapTime_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MeasureLapTime_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MeasureLapTime_GetResult_Response_type_support_ids_t;

static const _MeasureLapTime_GetResult_Response_type_support_ids_t _MeasureLapTime_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MeasureLapTime_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MeasureLapTime_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MeasureLapTime_GetResult_Response_type_support_symbol_names_t _MeasureLapTime_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, navigation_robot_custom_interfaces, action, MeasureLapTime_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_robot_custom_interfaces, action, MeasureLapTime_GetResult_Response)),
  }
};

typedef struct _MeasureLapTime_GetResult_Response_type_support_data_t
{
  void * data[2];
} _MeasureLapTime_GetResult_Response_type_support_data_t;

static _MeasureLapTime_GetResult_Response_type_support_data_t _MeasureLapTime_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MeasureLapTime_GetResult_Response_message_typesupport_map = {
  2,
  "navigation_robot_custom_interfaces",
  &_MeasureLapTime_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_MeasureLapTime_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_MeasureLapTime_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MeasureLapTime_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MeasureLapTime_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace navigation_robot_custom_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_GetResult_Response)() {
  return &::navigation_robot_custom_interfaces::action::rosidl_typesupport_c::MeasureLapTime_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MeasureLapTime_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MeasureLapTime_GetResult_type_support_ids_t;

static const _MeasureLapTime_GetResult_type_support_ids_t _MeasureLapTime_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MeasureLapTime_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MeasureLapTime_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MeasureLapTime_GetResult_type_support_symbol_names_t _MeasureLapTime_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, navigation_robot_custom_interfaces, action, MeasureLapTime_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_robot_custom_interfaces, action, MeasureLapTime_GetResult)),
  }
};

typedef struct _MeasureLapTime_GetResult_type_support_data_t
{
  void * data[2];
} _MeasureLapTime_GetResult_type_support_data_t;

static _MeasureLapTime_GetResult_type_support_data_t _MeasureLapTime_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MeasureLapTime_GetResult_service_typesupport_map = {
  2,
  "navigation_robot_custom_interfaces",
  &_MeasureLapTime_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_MeasureLapTime_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_MeasureLapTime_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t MeasureLapTime_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MeasureLapTime_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace navigation_robot_custom_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_GetResult)() {
  return &::navigation_robot_custom_interfaces::action::rosidl_typesupport_c::MeasureLapTime_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__struct.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _MeasureLapTime_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _MeasureLapTime_FeedbackMessage_type_support_ids_t;

static const _MeasureLapTime_FeedbackMessage_type_support_ids_t _MeasureLapTime_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _MeasureLapTime_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _MeasureLapTime_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _MeasureLapTime_FeedbackMessage_type_support_symbol_names_t _MeasureLapTime_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, navigation_robot_custom_interfaces, action, MeasureLapTime_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, navigation_robot_custom_interfaces, action, MeasureLapTime_FeedbackMessage)),
  }
};

typedef struct _MeasureLapTime_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _MeasureLapTime_FeedbackMessage_type_support_data_t;

static _MeasureLapTime_FeedbackMessage_type_support_data_t _MeasureLapTime_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _MeasureLapTime_FeedbackMessage_message_typesupport_map = {
  2,
  "navigation_robot_custom_interfaces",
  &_MeasureLapTime_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_MeasureLapTime_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_MeasureLapTime_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t MeasureLapTime_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_MeasureLapTime_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace navigation_robot_custom_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_FeedbackMessage)() {
  return &::navigation_robot_custom_interfaces::action::rosidl_typesupport_c::MeasureLapTime_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "navigation_robot_custom_interfaces/action/measure_lap_time.h"
// already included above
// #include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__type_support.h"

static rosidl_action_type_support_t _navigation_robot_custom_interfaces__action__MeasureLapTime__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime)()
{
  // Thread-safe by always writing the same values to the static struct
  _navigation_robot_custom_interfaces__action__MeasureLapTime__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_SendGoal)();
  _navigation_robot_custom_interfaces__action__MeasureLapTime__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_GetResult)();
  _navigation_robot_custom_interfaces__action__MeasureLapTime__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _navigation_robot_custom_interfaces__action__MeasureLapTime__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, navigation_robot_custom_interfaces, action, MeasureLapTime_FeedbackMessage)();
  _navigation_robot_custom_interfaces__action__MeasureLapTime__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_navigation_robot_custom_interfaces__action__MeasureLapTime__typesupport_c;
}

#ifdef __cplusplus
}
#endif
