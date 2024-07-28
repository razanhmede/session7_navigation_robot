

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FindClosestWall_Request {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for FindClosestWall_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::FindClosestWall_Request::default())
  }
}

impl rosidl_runtime_rs::Message for FindClosestWall_Request {
  type RmwMsg = crate::srv::rmw::FindClosestWall_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FindClosestWall_Response {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}



impl Default for FindClosestWall_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(crate::srv::rmw::FindClosestWall_Response::default())
  }
}

impl rosidl_runtime_rs::Message for FindClosestWall_Response {
  type RmwMsg = crate::srv::rmw::FindClosestWall_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        x: msg.x,
        y: msg.y,
        z: msg.z,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      x: msg.x,
      y: msg.y,
      z: msg.z,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      x: msg.x,
      y: msg.y,
      z: msg.z,
    }
  }
}






#[link(name = "navigation_robot_custom_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__navigation_robot_custom_interfaces__srv__FindClosestWall() -> *const std::os::raw::c_void;
}

// Corresponds to navigation_robot_custom_interfaces__srv__FindClosestWall
pub struct FindClosestWall;

impl rosidl_runtime_rs::Service for FindClosestWall {
  type Request = crate::srv::FindClosestWall_Request;
  type Response = crate::srv::FindClosestWall_Response;

  fn get_type_support() -> *const std::os::raw::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_service_type_support_handle__navigation_robot_custom_interfaces__srv__FindClosestWall() }
  }
}



pub mod rmw {
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[link(name = "navigation_robot_custom_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__navigation_robot_custom_interfaces__srv__FindClosestWall_Request() -> *const std::os::raw::c_void;
}

#[link(name = "navigation_robot_custom_interfaces__rosidl_generator_c")]
extern "C" {
    fn navigation_robot_custom_interfaces__srv__FindClosestWall_Request__init(msg: *mut FindClosestWall_Request) -> bool;
    fn navigation_robot_custom_interfaces__srv__FindClosestWall_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FindClosestWall_Request>, size: usize) -> bool;
    fn navigation_robot_custom_interfaces__srv__FindClosestWall_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FindClosestWall_Request>);
    fn navigation_robot_custom_interfaces__srv__FindClosestWall_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FindClosestWall_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<FindClosestWall_Request>) -> bool;
}

// Corresponds to navigation_robot_custom_interfaces__srv__FindClosestWall_Request
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FindClosestWall_Request {
    pub structure_needs_at_least_one_member: u8,
}



impl Default for FindClosestWall_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !navigation_robot_custom_interfaces__srv__FindClosestWall_Request__init(&mut msg as *mut _) {
        panic!("Call to navigation_robot_custom_interfaces__srv__FindClosestWall_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FindClosestWall_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_robot_custom_interfaces__srv__FindClosestWall_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_robot_custom_interfaces__srv__FindClosestWall_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_robot_custom_interfaces__srv__FindClosestWall_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FindClosestWall_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FindClosestWall_Request where Self: Sized {
  const TYPE_NAME: &'static str = "navigation_robot_custom_interfaces/srv/FindClosestWall_Request";
  fn get_type_support() -> *const std::os::raw::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__navigation_robot_custom_interfaces__srv__FindClosestWall_Request() }
  }
}


#[link(name = "navigation_robot_custom_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__navigation_robot_custom_interfaces__srv__FindClosestWall_Response() -> *const std::os::raw::c_void;
}

#[link(name = "navigation_robot_custom_interfaces__rosidl_generator_c")]
extern "C" {
    fn navigation_robot_custom_interfaces__srv__FindClosestWall_Response__init(msg: *mut FindClosestWall_Response) -> bool;
    fn navigation_robot_custom_interfaces__srv__FindClosestWall_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<FindClosestWall_Response>, size: usize) -> bool;
    fn navigation_robot_custom_interfaces__srv__FindClosestWall_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<FindClosestWall_Response>);
    fn navigation_robot_custom_interfaces__srv__FindClosestWall_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<FindClosestWall_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<FindClosestWall_Response>) -> bool;
}

// Corresponds to navigation_robot_custom_interfaces__srv__FindClosestWall_Response
#[repr(C)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct FindClosestWall_Response {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}



impl Default for FindClosestWall_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !navigation_robot_custom_interfaces__srv__FindClosestWall_Response__init(&mut msg as *mut _) {
        panic!("Call to navigation_robot_custom_interfaces__srv__FindClosestWall_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for FindClosestWall_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_robot_custom_interfaces__srv__FindClosestWall_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_robot_custom_interfaces__srv__FindClosestWall_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { navigation_robot_custom_interfaces__srv__FindClosestWall_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for FindClosestWall_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for FindClosestWall_Response where Self: Sized {
  const TYPE_NAME: &'static str = "navigation_robot_custom_interfaces/srv/FindClosestWall_Response";
  fn get_type_support() -> *const std::os::raw::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__navigation_robot_custom_interfaces__srv__FindClosestWall_Response() }
  }
}






  #[link(name = "navigation_robot_custom_interfaces__rosidl_typesupport_c")]
  extern "C" {
      fn rosidl_typesupport_c__get_service_type_support_handle__navigation_robot_custom_interfaces__srv__FindClosestWall() -> *const std::os::raw::c_void;
  }

  // Corresponds to navigation_robot_custom_interfaces__srv__FindClosestWall
  pub struct FindClosestWall;

  impl rosidl_runtime_rs::Service for FindClosestWall {
    type Request = crate::srv::rmw::FindClosestWall_Request;
    type Response = crate::srv::rmw::FindClosestWall_Response;

    fn get_type_support() -> *const std::os::raw::c_void {
      // SAFETY: No preconditions for this function.
      unsafe { rosidl_typesupport_c__get_service_type_support_handle__navigation_robot_custom_interfaces__srv__FindClosestWall() }
    }
  }



}  // mod rmw
