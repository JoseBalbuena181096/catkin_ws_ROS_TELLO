// Generated by gencpp from file flock_msgs/Flip.msg
// DO NOT EDIT!


#ifndef FLOCK_MSGS_MESSAGE_FLIP_H
#define FLOCK_MSGS_MESSAGE_FLIP_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace flock_msgs
{
template <class ContainerAllocator>
struct Flip_
{
  typedef Flip_<ContainerAllocator> Type;

  Flip_()
    : flip_command(0)  {
    }
  Flip_(const ContainerAllocator& _alloc)
    : flip_command(0)  {
  (void)_alloc;
    }



   typedef uint8_t _flip_command_type;
  _flip_command_type flip_command;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(flip_forward)
  #undef flip_forward
#endif
#if defined(_WIN32) && defined(flip_left)
  #undef flip_left
#endif
#if defined(_WIN32) && defined(flip_back)
  #undef flip_back
#endif
#if defined(_WIN32) && defined(flip_right)
  #undef flip_right
#endif
#if defined(_WIN32) && defined(flip_forwardleft)
  #undef flip_forwardleft
#endif
#if defined(_WIN32) && defined(flip_backleft)
  #undef flip_backleft
#endif
#if defined(_WIN32) && defined(flip_backright)
  #undef flip_backright
#endif
#if defined(_WIN32) && defined(flip_forwardright)
  #undef flip_forwardright
#endif

  enum {
    flip_forward = 0u,
    flip_left = 1u,
    flip_back = 2u,
    flip_right = 3u,
    flip_forwardleft = 4u,
    flip_backleft = 5u,
    flip_backright = 6u,
    flip_forwardright = 7u,
  };


  typedef boost::shared_ptr< ::flock_msgs::Flip_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::flock_msgs::Flip_<ContainerAllocator> const> ConstPtr;

}; // struct Flip_

typedef ::flock_msgs::Flip_<std::allocator<void> > Flip;

typedef boost::shared_ptr< ::flock_msgs::Flip > FlipPtr;
typedef boost::shared_ptr< ::flock_msgs::Flip const> FlipConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::flock_msgs::Flip_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::flock_msgs::Flip_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::flock_msgs::Flip_<ContainerAllocator1> & lhs, const ::flock_msgs::Flip_<ContainerAllocator2> & rhs)
{
  return lhs.flip_command == rhs.flip_command;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::flock_msgs::Flip_<ContainerAllocator1> & lhs, const ::flock_msgs::Flip_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace flock_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::flock_msgs::Flip_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::flock_msgs::Flip_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::flock_msgs::Flip_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::flock_msgs::Flip_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::flock_msgs::Flip_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::flock_msgs::Flip_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::flock_msgs::Flip_<ContainerAllocator> >
{
  static const char* value()
  {
    return "900021b0eeb6e5c98ddc735ee4fde90e";
  }

  static const char* value(const ::flock_msgs::Flip_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x900021b0eeb6e5c9ULL;
  static const uint64_t static_value2 = 0x8ddc735ee4fde90eULL;
};

template<class ContainerAllocator>
struct DataType< ::flock_msgs::Flip_<ContainerAllocator> >
{
  static const char* value()
  {
    return "flock_msgs/Flip";
  }

  static const char* value(const ::flock_msgs::Flip_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::flock_msgs::Flip_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Names and values match TelloPy\n"
"uint8 flip_forward=0\n"
"uint8 flip_left=1\n"
"uint8 flip_back=2\n"
"uint8 flip_right=3\n"
"uint8 flip_forwardleft=4\n"
"uint8 flip_backleft=5\n"
"uint8 flip_backright=6\n"
"uint8 flip_forwardright=7\n"
"uint8 flip_command\n"
;
  }

  static const char* value(const ::flock_msgs::Flip_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::flock_msgs::Flip_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.flip_command);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Flip_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::flock_msgs::Flip_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::flock_msgs::Flip_<ContainerAllocator>& v)
  {
    s << indent << "flip_command: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.flip_command);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FLOCK_MSGS_MESSAGE_FLIP_H
