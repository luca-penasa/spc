#ifndef L_POINT_TYPES_H
#define L_POINT_TYPES_H

#include <pcl/point_types.h>

struct PointD //point strucure with just a distance value
{
	float distance;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointD,           // here we assume a XYZ + "test" (as fields)
(float, distance, distance)
)

struct PointI //point strucure with just a distance value
{
	float intensity;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointI,           // here we assume a XYZ + "test" (as fields)
(float, intensity, intensity)
)

struct PointIntDist //point strucure with just a distance value
{
	float intensity;
	float distance;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointIntDist,           // here we assume a XYZ + "test" (as fields)
(float, distance, distance)
(float, intensity, intensity)
)

#endif


