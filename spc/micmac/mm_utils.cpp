
#include "mm_utils.h"
#include "general/geom_vecteur.h"

namespace spc
{
int imageToModelSpace(const Pt2dr &pt2d, const cElNuage3DMaille &aNuage,
                      Pt3dr &pt3d)
{
    // NOTE we are assuming the pt2d that the user passes is ensured to be
    // inside the image's dimensions!!
    // we should add a check for this

    // aNuage.ImRef2Capteur()
    // aNuage.PIsVisibleInImage() #chiede già un punto in 3d questo
    // aNuage.ResolImRefFromCapteur();
    // aNuage.Ter2Capteur()

    bool has_data = aNuage.CaptHasData(pt2d);

    // what does the mask say about this point?
    // bool exists = mask.GetI(point2d_int);
    if (!has_data) {
        pt3d.x = std::numeric_limits<float>::quiet_NaN();
        pt3d.y = std::numeric_limits<float>::quiet_NaN();
        pt3d.z = std::numeric_limits<float>::quiet_NaN();
        return 0;
    } else {
        pt3d = aNuage.PreciseCapteur2Terrain(pt2d);
        return 1;
    }
}

int modelToImageSpace(const Pt3dr &point, const ElCamera &cam, Pt2dr &im_point)
{

    //    Pt3dr aPG;

    //    Pt3dr aPC = cam->R3toL3(aPG);
    //    Pt2dr aIm1 = cam->R3toC2(aPG);
    //    Pt2dr aIm2 = cam->DComplC2M(aCam->R3toF2(aPG));
    //    Pt2dr aIm3 = cam->OrGlbImaC2M(aCam->R3toF2(aPG));
    im_point = cam.R3toF2(point);

    return cam.IsInZoneUtile(im_point);
}

statusesT multiPointsImageToModelSpace(const Pts2dr &in_points,
                                       const cElNuage3DMaille &aNuage,
                                       Pts3dr &out_points)
{
    // create return object
    statusesT statuses;
    // ensure output is clear
    out_points.clear();

    Pt3dr point3d; // as tmp object for storing data
    for (auto point : in_points) {
        statuses.push_back(imageToModelSpace(point, aNuage, point3d));
        out_points.push_back(point3d);
    }
    return statuses;
}

statusesT multiPointsModelToImageSpace(const Pts3dr &in_points,
                                       const ElCamera &cam, Pts2dr &out_points)
{
    // create return object
    statusesT statuses;
    // ensure output is clear
    out_points.clear();

    Pt2dr point; // as tmp object for storing data
    for (auto p : in_points) {
        statuses.push_back(modelToImageSpace(p, cam, point));
        out_points.push_back(point);
    }
    return statuses;
}

} // end nspace
