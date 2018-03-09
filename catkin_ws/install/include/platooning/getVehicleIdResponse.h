// Generated by gencpp from file platooning/getVehicleIdResponse.msg
// DO NOT EDIT!


#ifndef PLATOONING_MESSAGE_GETVEHICLEIDRESPONSE_H
#define PLATOONING_MESSAGE_GETVEHICLEIDRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace platooning
{
template <class ContainerAllocator>
struct getVehicleIdResponse_
{
  typedef getVehicleIdResponse_<ContainerAllocator> Type;

  getVehicleIdResponse_()
    : vehicle_id(0)  {
    }
  getVehicleIdResponse_(const ContainerAllocator& _alloc)
    : vehicle_id(0)  {
  (void)_alloc;
    }



   typedef uint32_t _vehicle_id_type;
  _vehicle_id_type vehicle_id;





  typedef boost::shared_ptr< ::platooning::getVehicleIdResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::platooning::getVehicleIdResponse_<ContainerAllocator> const> ConstPtr;

}; // struct getVehicleIdResponse_

typedef ::platooning::getVehicleIdResponse_<std::allocator<void> > getVehicleIdResponse;

typedef boost::shared_ptr< ::platooning::getVehicleIdResponse > getVehicleIdResponsePtr;
typedef boost::shared_ptr< ::platooning::getVehicleIdResponse const> getVehicleIdResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::platooning::getVehicleIdResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::platooning::getVehicleIdResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace platooning

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/lunar/share/std_msgs/cmake/../msg'], 'platooning': ['/home/stepo/workspace/autonomesfahren/Gruppe-C/catkin_ws/src/platooning/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::platooning::getVehicleIdResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::platooning::getVehicleIdResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::platooning::getVehicleIdResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::platooning::getVehicleIdResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::platooning::getVehicleIdResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::platooning::getVehicleIdResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::platooning::getVehicleIdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5c96f4b8297034815b56c1d26627cd99";
  }

  static const char* value(const ::platooning::getVehicleIdResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5c96f4b829703481ULL;
  static const uint64_t static_value2 = 0x5b56c1d26627cd99ULL;
};

template<class ContainerAllocator>
struct DataType< ::platooning::getVehicleIdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "platooning/getVehicleIdResponse";
  }

  static const char* value(const ::platooning::getVehicleIdResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::platooning::getVehicleIdResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 vehicle_id\n\
";
  }

  static const char* value(const ::platooning::getVehicleIdResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::platooning::getVehicleIdResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.vehicle_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct getVehicleIdResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::platooning::getVehicleIdResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::platooning::getVehicleIdResponse_<ContainerAllocator>& v)
  {
    s << indent << "vehicle_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.vehicle_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLATOONING_MESSAGE_GETVEHICLEIDRESPONSE_H
