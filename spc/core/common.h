#ifndef SPC_COMMON_H
#define SPC_COMMON_H

#include <spc/core/macros.h>

#ifdef SPC_WITH_PCL

#ifdef PCL_VER_LESS_1_7
#include <sensor_msgs/PointCloud2.h>

namespace pcl
{
typedef sensor_msgs::PointCloud2 PCLPointCloud2;
typedef sensor_msgs::PointField PCLPointField;
typedef std_msgs::Header PCLHeader;

#define fromPCLPointCloud2 fromROSMsg
#define toPCLPointCloud2 toROSMsg
}

#else
#include <pcl/PCLPointCloud2.h>
#endif

#endif //SPC_WITH_PCL


#if defined(_WIN32)

#include <stdint.h>

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;

#endif

#endif
