#include "StdAfx.h"
//#include <general/ptxd.h>
//#include "general/photogram.h"

#include "mm_utils.h"

//void TestDirect(ElCamera * aCam,Pt3dr aPG)
//{
//    {
//         std::cout << " ---PGround  = " << aPG << "\n";

//         Pt3dr aPC = aCam->R3toL3(aPG);
//         std::cout << " -0-CamCoord = " << aPC << "\n";

//         Pt2dr aIm1 = aCam->R3toC2(aPG);
//         std::cout << " -1-ImSsDist = " << aIm1 << "\n";


//         Pt2dr aIm2 = aCam->DComplC2M(aCam->R3toF2(aPG));
//         std::cout << " -2-ImDist 1 = " << aIm2 << "\n";

//         Pt2dr aIm3 = aCam->OrGlbImaC2M(aCam->R3toF2(aPG));
//         std::cout << " -3-ImDist N = " << aIm3 << "\n";

//         Pt2dr aIm4 = aCam->R3toF2(aPG);
//         std::cout << " -4-ImFinale = " << aIm4 << "\n";
//    }
//}

int main(int argc, char ** argv)
{
    std::string model3d_file = argv[1];
    float pix_x = atof(argv[2]);
    float pix_y = atof(argv[3]);


    cElNuage3DMaille *  aNuage = cElNuage3DMaille::FromFileIm(model3d_file);


    Pt2dr point (pix_x, pix_y);
    Pt3dr point3d;
    int status = imageToModelSpace(point, *aNuage, point3d);

    std::cout << status << " " << point3d << std::endl;

    //now try with multi points
    Pts2dr inpoints;
    Pts3dr outpoints;

    inpoints.push_back(point);
    point.x = 900;
    point.y = 900;
    inpoints.push_back(point);
    point.x = 3200;
    point.y = 1200;
    inpoints.push_back(point);

    statusesT statuses = multiPointsImageToModelSpace(inpoints, *aNuage, outpoints);

    for (auto point : outpoints)
        std::cout << point << std::endl;

    for (auto status : statuses)
        std::cout << status << std::endl;



    return 1;
}

