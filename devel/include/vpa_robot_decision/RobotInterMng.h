// Generated by gencpp from file vpa_robot_decision/RobotInterMng.msg
// DO NOT EDIT!


#ifndef VPA_ROBOT_DECISION_MESSAGE_ROBOTINTERMNG_H
#define VPA_ROBOT_DECISION_MESSAGE_ROBOTINTERMNG_H

#include <ros/service_traits.h>


#include <vpa_robot_decision/RobotInterMngRequest.h>
#include <vpa_robot_decision/RobotInterMngResponse.h>


namespace vpa_robot_decision
{

struct RobotInterMng
{

typedef RobotInterMngRequest Request;
typedef RobotInterMngResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RobotInterMng
} // namespace vpa_robot_decision


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::vpa_robot_decision::RobotInterMng > {
  static const char* value()
  {
    return "c0abed4502d76f69bbb34192950a96ca";
  }

  static const char* value(const ::vpa_robot_decision::RobotInterMng&) { return value(); }
};

template<>
struct DataType< ::vpa_robot_decision::RobotInterMng > {
  static const char* value()
  {
    return "vpa_robot_decision/RobotInterMng";
  }

  static const char* value(const ::vpa_robot_decision::RobotInterMng&) { return value(); }
};


// service_traits::MD5Sum< ::vpa_robot_decision::RobotInterMngRequest> should match
// service_traits::MD5Sum< ::vpa_robot_decision::RobotInterMng >
template<>
struct MD5Sum< ::vpa_robot_decision::RobotInterMngRequest>
{
  static const char* value()
  {
    return MD5Sum< ::vpa_robot_decision::RobotInterMng >::value();
  }
  static const char* value(const ::vpa_robot_decision::RobotInterMngRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::vpa_robot_decision::RobotInterMngRequest> should match
// service_traits::DataType< ::vpa_robot_decision::RobotInterMng >
template<>
struct DataType< ::vpa_robot_decision::RobotInterMngRequest>
{
  static const char* value()
  {
    return DataType< ::vpa_robot_decision::RobotInterMng >::value();
  }
  static const char* value(const ::vpa_robot_decision::RobotInterMngRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::vpa_robot_decision::RobotInterMngResponse> should match
// service_traits::MD5Sum< ::vpa_robot_decision::RobotInterMng >
template<>
struct MD5Sum< ::vpa_robot_decision::RobotInterMngResponse>
{
  static const char* value()
  {
    return MD5Sum< ::vpa_robot_decision::RobotInterMng >::value();
  }
  static const char* value(const ::vpa_robot_decision::RobotInterMngResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::vpa_robot_decision::RobotInterMngResponse> should match
// service_traits::DataType< ::vpa_robot_decision::RobotInterMng >
template<>
struct DataType< ::vpa_robot_decision::RobotInterMngResponse>
{
  static const char* value()
  {
    return DataType< ::vpa_robot_decision::RobotInterMng >::value();
  }
  static const char* value(const ::vpa_robot_decision::RobotInterMngResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // VPA_ROBOT_DECISION_MESSAGE_ROBOTINTERMNG_H
