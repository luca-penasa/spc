#include "StdAfx.h"
//#include <general/ptxd.h>
//#include "general/photogram.h"

void TestDirect(ElCamera * aCam,Pt3dr aPG)
{
    {
         std::cout << " ---PGround  = " << aPG << "\n";

         Pt3dr aPC = aCam->R3toL3(aPG);
         std::cout << " -0-CamCoord = " << aPC << "\n";

         Pt2dr aIm1 = aCam->R3toC2(aPG);
         std::cout << " -1-ImSsDist = " << aIm1 << "\n";


         Pt2dr aIm2 = aCam->DComplC2M(aCam->R3toF2(aPG));
         std::cout << " -2-ImDist 1 = " << aIm2 << "\n";

         Pt2dr aIm3 = aCam->OrGlbImaC2M(aCam->R3toF2(aPG));
         std::cout << " -3-ImDist N = " << aIm3 << "\n";

         Pt2dr aIm4 = aCam->R3toF2(aPG);
         std::cout << " -4-ImFinale = " << aIm4 << "\n";
    }
}

int main(int argc, char ** argv)
{
    std::string aFullName;
    std::string aNameCam;
    std::string aNameDir;
    std::string aNameTag = "OrientationConique";

    double X,Y,Z;
    bool aModeGrid = false;
    std::string Out;


    ElInitArgMain
    (
    argc,argv,
    LArgMain()  << EAMC(aFullName,"Name")
                << EAMC(X,"x")
                << EAMC(Y,"y")
                << EAMC(Z,"z"),
    LArgMain()
                    << EAM(aNameTag,"Tag",true,"Tag to get cam")
                    << EAM(aModeGrid,"Grid",true,"Test Grid Mode")
                    << EAM(Out,"Out",true,"To Regenerate an orientation file")
    );

    SplitDirAndFile(aNameDir,aNameCam,aFullName);

    cTplValGesInit<std::string>  aTplFCND;
    cInterfChantierNameManipulateur * anICNM = cInterfChantierNameManipulateur::BasicAlloc(aNameDir);
/*
    cInterfChantierNameManipulateur * anICNM =
        cInterfChantierNameManipulateur::StdAlloc(0,0,aNameDir,aTplFCND);
*/


   ElCamera * aCam  = Gen_Cam_Gen_From_File(aModeGrid,aFullName,aNameTag,anICNM);

   std::string filename = "/media/Data/cavaPaolo/models/model_F028_D04_T163241.xml";
   cElNuage3DMaille *  aNuage = cElNuage3DMaille::FromFileIm(filename);
   Im2D_Bits<1> mask = aNuage->ImMask();




   double pix_x, pix_y;
   pix_x = 3097;
   pix_y = 996;
   Pt2di point_int (pix_x, pix_y);
   Pt2dr point(pix_x, pix_y);

   bool status_1 = mask.GetI(point_int);

   mask.AssertInside(point_int);

   bool status = aNuage->HasPreciseCapteur2Terrain();
   Pt3dr point3d;
   point3d = aNuage->PreciseCapteur2Terrain(point);
   std::cout << "Has capture: " << status << std::endl;
   std::cout << "get status: " << status_1 << std::endl;
   std::cout << "Back-projection: " << point3d << std::endl;

    return 0;
}
