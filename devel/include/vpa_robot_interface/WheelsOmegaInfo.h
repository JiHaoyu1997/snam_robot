// Generated by gencpp from file vpa_robot_interface/WheelsOmegaInfo.msg
// DO NOT EDIT!


#ifndef VPA_ROBOT_INTERFACE_MESSAGE_WHEELSOMEGAINFO_H
#define VPA_ROBOT_INTERFACE_MESSAGE_WHEELSOMEGAINFO_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vpa_robot_interface
{
template <class ContainerAllocator>
struct WheelsOmegaInfo_
{
  typedef WheelsOmegaInfo_<ContainerAllocator> Type;

  WheelsOmegaInfo_()
    : omega_left_ref(0.0)
    , omega_left_sig(0.0)
    , omega_right_ref(0.0)
    , omega_right_sig(0.0)  {
    }
  WheelsOmegaInfo_(const ContainerAllocator& _alloc)
    : omega_left_ref(0.0)
    , omega_left_sig(0.0)
    , omega_right_ref(0.0)
    , omega_right_sig(0.0)  {
  (void)_alloc;
    }



   typedef float _omega_left_ref_type;
  _omega_left_ref_type omega_left_ref;

   typedef float _omega_left_sig_type;
  _omega_left_sig_type omega_left_sig;

   typedef float _omega_right_ref_type;
  _omega_right_ref_type omega_right_ref;

   typedef float _omega_right_sig_type;
  _omega_right_sig_type omega_right_sig;





  typedef boost::shared_ptr< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> const> ConstPtr;

}; // struct WheelsOmegaInfo_

typedef ::vpa_robot_interface::WheelsOmegaInfo_<std::allocator<void> > WheelsOmegaInfo;

typedef boost::shared_ptr< ::vpa_robot_interface::WheelsOmegaInfo > WheelsOmegaInfoPtr;
typedef boost::shared_ptr< ::vpa_robot_interface::WheelsOmegaInfo const> WheelsOmegaInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator1> & lhs, const ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator2> & rhs)
{
  return lhs.omega_left_ref == rhs.omega_left_ref &&
    lhs.omega_left_sig == rhs.omega_left_sig &&
    lhs.omega_right_ref == rhs.omega_right_ref &&
    lhs.omega_right_sig == rhs.omega_right_sig;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator1> & lhs, const ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vpa_robot_interface

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ba80e47329fa708db7e2fff0c002fd00";
  }

  static const char* value(const ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xba80e47329fa708dULL;
  static const uint64_t static_value2 = 0xb7e2fff0c002fd00ULL;
};

template<class ContainerAllocator>
struct DataType< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vpa_robot_interface/WheelsOmegaInfo";
  }

  static const char* value(const ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 omega_left_ref\n"
"float32 omega_left_sig\n"
"float32 omega_right_ref\n"
"float32 omega_right_sig\n"
;
  }

  static const char* value(const ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.omega_left_ref);
      stream.next(m.omega_left_sig);
      stream.next(m.omega_right_ref);
      stream.next(m.omega_right_sig);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WheelsOmegaInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vpa_robot_interface::WheelsOmegaInfo_<ContainerAllocator>& v)
  {
    s << indent << "omega_left_ref: ";
    Printer<float>::stream(s, indent + "  ", v.omega_left_ref);
    s << indent << "omega_left_sig: ";
    Printer<float>::stream(s, indent + "  ", v.omega_left_sig);
    s << indent << "omega_right_ref: ";
    Printer<float>::stream(s, indent + "  ", v.omega_right_ref);
    s << indent << "omega_right_sig: ";
    Printer<float>::stream(s, indent + "  ", v.omega_right_sig);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VPA_ROBOT_INTERFACE_MESSAGE_WHEELSOMEGAINFO_H
