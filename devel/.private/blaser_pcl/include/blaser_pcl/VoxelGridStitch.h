// Generated by gencpp from file blaser_pcl/VoxelGridStitch.msg
// DO NOT EDIT!


#ifndef BLASER_PCL_MESSAGE_VOXELGRIDSTITCH_H
#define BLASER_PCL_MESSAGE_VOXELGRIDSTITCH_H

#include <ros/service_traits.h>


#include <blaser_pcl/VoxelGridStitchRequest.h>
#include <blaser_pcl/VoxelGridStitchResponse.h>


namespace blaser_pcl
{

struct VoxelGridStitch
{

typedef VoxelGridStitchRequest Request;
typedef VoxelGridStitchResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct VoxelGridStitch
} // namespace blaser_pcl


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::blaser_pcl::VoxelGridStitch > {
  static const char* value()
  {
    return "60c18bc0467086605bc6bd7bde5154dd";
  }

  static const char* value(const ::blaser_pcl::VoxelGridStitch&) { return value(); }
};

template<>
struct DataType< ::blaser_pcl::VoxelGridStitch > {
  static const char* value()
  {
    return "blaser_pcl/VoxelGridStitch";
  }

  static const char* value(const ::blaser_pcl::VoxelGridStitch&) { return value(); }
};


// service_traits::MD5Sum< ::blaser_pcl::VoxelGridStitchRequest> should match 
// service_traits::MD5Sum< ::blaser_pcl::VoxelGridStitch > 
template<>
struct MD5Sum< ::blaser_pcl::VoxelGridStitchRequest>
{
  static const char* value()
  {
    return MD5Sum< ::blaser_pcl::VoxelGridStitch >::value();
  }
  static const char* value(const ::blaser_pcl::VoxelGridStitchRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::blaser_pcl::VoxelGridStitchRequest> should match 
// service_traits::DataType< ::blaser_pcl::VoxelGridStitch > 
template<>
struct DataType< ::blaser_pcl::VoxelGridStitchRequest>
{
  static const char* value()
  {
    return DataType< ::blaser_pcl::VoxelGridStitch >::value();
  }
  static const char* value(const ::blaser_pcl::VoxelGridStitchRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::blaser_pcl::VoxelGridStitchResponse> should match 
// service_traits::MD5Sum< ::blaser_pcl::VoxelGridStitch > 
template<>
struct MD5Sum< ::blaser_pcl::VoxelGridStitchResponse>
{
  static const char* value()
  {
    return MD5Sum< ::blaser_pcl::VoxelGridStitch >::value();
  }
  static const char* value(const ::blaser_pcl::VoxelGridStitchResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::blaser_pcl::VoxelGridStitchResponse> should match 
// service_traits::DataType< ::blaser_pcl::VoxelGridStitch > 
template<>
struct DataType< ::blaser_pcl::VoxelGridStitchResponse>
{
  static const char* value()
  {
    return DataType< ::blaser_pcl::VoxelGridStitch >::value();
  }
  static const char* value(const ::blaser_pcl::VoxelGridStitchResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // BLASER_PCL_MESSAGE_VOXELGRIDSTITCH_H
