// Generated by gencpp from file prius_msgs/Speed.msg
// DO NOT EDIT!


#ifndef PRIUS_MSGS_MESSAGE_SPEED_H
#define PRIUS_MSGS_MESSAGE_SPEED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace prius_msgs
{
template <class ContainerAllocator>
struct Speed_
{
  typedef Speed_<ContainerAllocator> Type;

  Speed_()
    : speed(0.0)  {
    }
  Speed_(const ContainerAllocator& _alloc)
    : speed(0.0)  {
  (void)_alloc;
    }



   typedef float _speed_type;
  _speed_type speed;





  typedef boost::shared_ptr< ::prius_msgs::Speed_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::prius_msgs::Speed_<ContainerAllocator> const> ConstPtr;

}; // struct Speed_

typedef ::prius_msgs::Speed_<std::allocator<void> > Speed;

typedef boost::shared_ptr< ::prius_msgs::Speed > SpeedPtr;
typedef boost::shared_ptr< ::prius_msgs::Speed const> SpeedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::prius_msgs::Speed_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::prius_msgs::Speed_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace prius_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/lunar/share/std_msgs/cmake/../msg'], 'prius_msgs': ['/home/stepo/workspace/autonomesfahren/Gruppe-C/catkin_ws/src/prius_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::prius_msgs::Speed_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::prius_msgs::Speed_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::prius_msgs::Speed_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::prius_msgs::Speed_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::prius_msgs::Speed_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::prius_msgs::Speed_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::prius_msgs::Speed_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ca65bba734a79b4a6707341d829f4d5c";
  }

  static const char* value(const ::prius_msgs::Speed_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xca65bba734a79b4aULL;
  static const uint64_t static_value2 = 0x6707341d829f4d5cULL;
};

template<class ContainerAllocator>
struct DataType< ::prius_msgs::Speed_<ContainerAllocator> >
{
  static const char* value()
  {
    return "prius_msgs/Speed";
  }

  static const char* value(const ::prius_msgs::Speed_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::prius_msgs::Speed_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 speed\n\
";
  }

  static const char* value(const ::prius_msgs::Speed_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::prius_msgs::Speed_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.speed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Speed_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::prius_msgs::Speed_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::prius_msgs::Speed_<ContainerAllocator>& v)
  {
    s << indent << "speed: ";
    Printer<float>::stream(s, indent + "  ", v.speed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PRIUS_MSGS_MESSAGE_SPEED_H