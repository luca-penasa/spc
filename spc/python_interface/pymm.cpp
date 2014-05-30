#include "pymm.h"

namespace spc
{

char const *pythonStr2cstr(boost::python::str string)
{
    return boost::python::extract<char const *>(string);
}

boost::numpy::ndarray fromImageToModelSpace(boost::numpy::ndarray pts2d,
                                            boost::python::str nuage_filename)
{

    boost::python::list *out = new boost::python::list;
    Pts2dr points2d;
    Pts3dr points3d;

    int n_points = boost::python::len(pts2d);

    for (size_t i = 0; i < n_points; ++i)
        points2d.push_back(Pt2dr(boost::python::extract<double>(pts2d[i][0]),
                                 boost::python::extract<double>(pts2d[i][1])));

    cElNuage3DMaille *aNuage
        = cElNuage3DMaille::FromFileIm(pythonStr2cstr(nuage_filename));

    statusesT s = multiPointsImageToModelSpace(points2d, *aNuage, points3d);

    for (size_t i = 0; i < n_points; ++i) {
        boost::python::list point3d;
        point3d.append(points3d.at(i).x);
        point3d.append(points3d.at(i).y);
        point3d.append(points3d.at(i).z);
        point3d.append(s.at(i));

        out->append(point3d);
    }

    boost::numpy::ndarray array = boost::numpy::array(*out);

    return array;
}

boost::numpy::ndarray
fromModelToImageSpace(boost::numpy::ndarray pts3d,
                      boost::python::str orientation_filename)
{
    std::string aNameTag = "OrientationConique";
    std::string aNameDir;
    std::string aNameCam;

    SplitDirAndFile(aNameDir, aNameCam, pythonStr2cstr(orientation_filename));

    cInterfChantierNameManipulateur *anICNM
        = cInterfChantierNameManipulateur::BasicAlloc(aNameDir);

    ElCamera *cam = Gen_Cam_Gen_From_File(
        true, pythonStr2cstr(orientation_filename), aNameTag, anICNM);

    int n_points = boost::python::len(pts3d);

    Pts3dr points3d;
    Pts2dr points2d;

    for (size_t i = 0; i < n_points; ++i)
        points3d.push_back(Pt3dr(boost::python::extract<double>(pts3d[i][0]),
                                 boost::python::extract<double>(pts3d[i][1]),
                                 boost::python::extract<double>(pts3d[i][2])));

    statusesT s = multiPointsModelToImageSpace(points3d, *cam, points2d);

    // compose the output list
    boost::python::list *list = new boost::python::list;

    for (size_t i = 0; i < n_points; ++i) {
        boost::python::list pt2d;
        pt2d.append(points2d.at(i).x);
        pt2d.append(points2d.at(i).y);
        pt2d.append(s.at(i));

        list->append(pt2d);
    }

    boost::numpy::ndarray array = boost::numpy::array(*list);
    return array;
}

BOOST_PYTHON_MODULE(libpyspc)
{
    boost::numpy::initialize();

    using namespace boost::numpy;

    def("micmac.fromImageToModelSpace", fromImageToModelSpace);
    def("micmac.fromModelToImageSpace", fromModelToImageSpace);
}

} // end nspace
