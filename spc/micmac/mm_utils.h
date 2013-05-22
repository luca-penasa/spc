#ifndef MM_UTILS_H
#define MM_UTILS_H

#include <StdAfx.h>
#include <general/ptxd.h>
#include <vector>

namespace spc
{
typedef std::vector<Pt2dr> Pts2dr;
typedef std::vector<Pt3dr> Pts3dr;
typedef std::vector<int> statusesT;

// SINGLE POINT ////////////////////////////////////////////
int imageToModelSpace(const Pt2dr &pt2d, const cElNuage3DMaille  &aNuage, Pt3dr &pt3d);

int modelToImageSpace( const Pt3dr &point, const ElCamera &cam, Pt2dr &im_point);


// MULTI POINTS ////////////////////////////////////////////
statusesT multiPointsImageToModelSpace(const Pts2dr &in_points , const cElNuage3DMaille &aNuage, Pts3dr &out_points);

statusesT multiPointsModelToImageSpace(const Pts3dr &in_points, const ElCamera &cam, Pts2dr &out_points);

}//end nspace
#endif // MM_UTILS_H
