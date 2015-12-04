#include "elements.h"
#include <spc/core/spc_eigen.h>

#include <boost/python.hpp>
#include "eigen_numpy.h"


#include "indexing_suite_v2/indexing_suite/container_suite.hpp"
#include "indexing_suite_v2/indexing_suite/vector.hpp"


#include <spc/elements/calibration/DataHolder.h>
#include <spc/elements/calibration/KeyPoint.h>






#include <spc/methods/RBFModelEstimator.h>




using namespace boost::python;
using namespace spc::calibration;

typedef spc::RBFModel<float> RBFModelF;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(op, RBFModelF::operator(), 1, 1)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(getFieldByName, NewSpcPointCloud::getFieldByName, 1, 1)


void test_function(const std::vector <std::string> &in)
{
    for (auto i : in)
    {
        LOG(INFO) << i;
    }
}

template<typename T>
void wrap_std_vector(const std::string name)
{
	typedef std::vector <T> VT;
	class_< VT>(name.c_str())
			.def(indexing::container_suite<VT >())
			;
}




template<typename T1, typename T2>
void wrap_std_map(const std::string name)
{
	typedef std::map <T1, T2> VT;
	class_< VT>(name.c_str())
			.def(indexing::container_suite<VT >())
			;
}

BOOST_PYTHON_MODULE(elements)
{
    implicitly_convertible<RBFModelF::Ptr,ISerializable::Ptr>();
	implicitly_convertible<RBFModelF::Ptr,ElementBase::Ptr>();
	implicitly_convertible<ElementBase::Ptr,ISerializable::Ptr>();

	wrap_std_vector<ElementBase::Ptr> ("VectorOfElementBasePtr");
	wrap_std_vector<Observation::Ptr> ("VectorOfObservationPtr");
	wrap_std_vector<CloudDataSourceOnDisk::Ptr> ("VectorOfloudDataSourceOnDiskPtr");


    def("testFunction", &test_function);

//    class_<std::vector<std::string, std::allocator<std::string> > > ("VectorOfStrings2")
//            .def(indexing::container_suite< std::vector<std::string, std::allocator<std::string> > >())
//            ;

    wrap_std_vector<std::string> ("VectorOfStrings");


	class_<ISerializable, ISerializable::Ptr, boost::noncopyable>("ISerializable", no_init)
			.def("isSerializable", &ISerializable::isSerializable)
			.def("isAsciiSerializable", &ISerializable::isAsciiSerializable);
			;

	class_<ElementBase, ElementBase::Ptr, boost::noncopyable, bases<ISerializable> >("ElementBase", no_init)
			.def("getPtr", &ElementBase::getPtr)
			.def("clone", pure_virtual(&ElementBase::clone))
			.def("getChilds", &ElementBase::getChilds)
			.def("addChild", &ElementBase::addChild)
			.def("getElementName", &ElementBase::getElementName)
			.def("setElementName", &ElementBase::setElementName)
			.def("removeChild", &ElementBase::removeChild)
			.def("setParent", &ElementBase::getParent)
			;

    enum_<RBFKernelFactory<float>::RBF_FUNCTION>("RBF_FUNCTION")
            .value("RBF_GAUSSIAN", RBFKernelFactory<float>::RBF_GAUSSIAN)
            .value("RBF_GAUSSIAN_APPROX", RBFKernelFactory<float>::RBF_GAUSSIAN_APPROX)
            .value("RBF_MULTIQUADRIC", RBFKernelFactory<float>::RBF_MULTIQUADRIC)
            .value("RBF_EPANECHNIKOV", RBFKernelFactory<float>::RBF_EPANECHNIKOV)
            .value("RBF_POLYHARMONIC_1", RBFKernelFactory<float>::RBF_POLYHARMONIC_1)
            .value("RBF_POLYHARMONIC_2", RBFKernelFactory<float>::RBF_POLYHARMONIC_2)
            .value("RBF_POLYHARMONIC_3", RBFKernelFactory<float>::RBF_POLYHARMONIC_3)
            .value("RBF_POLYHARMONIC_4", RBFKernelFactory<float>::RBF_POLYHARMONIC_4)
            .value("RBF_POLYHARMONIC_5", RBFKernelFactory<float>::RBF_POLYHARMONIC_5)
            .value("RBF_POLYHARMONIC_6", RBFKernelFactory<float>::RBF_POLYHARMONIC_6)


            ;



    class_<RBFKernelFactory<float> > ("RBFModelFactoryF")
            .def("create", &RBFKernelFactory<float>::create)
            .staticmethod("create")
            ;


    class_<RBFBase<float>, RBFBase<float>::Ptr, bases<ElementBase>,  boost::noncopyable> ("RBFBaseF", no_init);

	class_<RBFModelF,RBFModelF::Ptr, bases<ElementBase>>("RBFModel")
			.def("getCoefficients", &RBFModelF::getCoefficients)
            .def("getSigma", &RBFModelF::getSigma)
            .def("setSigma", &RBFModelF::setSigma)
            .def("getNumberOfNodes", &RBFModelF::getNumberOfNodes)
            .def("getNodes", &RBFModelF::getNodes)

            .def("getNumberOfpolynomialTerms", &RBFModelF::getNumberOfpolynomialTerms)
            .def("setPolyOrder", &RBFModelF::setPolyOrder)
            .def("getPolyOrder", &RBFModelF::getPolyOrder)
            .def("getKernel", &RBFModelF::getKernel)
            .def("setKernel", &RBFModelF::setKernel)

			.def("getDimensionality", &RBFModelF::getDimensionality)
//			.def("__call__",  static_cast<float (RBFModelF::*)(const RBFModelF::PointT &) const >  (&RBFModelF::operator() ), op() )
            .def("clone", &RBFModelF::clone)
//			.def("__call__",  static_cast<float (RBFModelF::*)(const RBFModelF::PointT &) const >  (&RBFModelF::operator() ), op() )
            .def("__call__", &RBFModelF::eval)

			;

	implicitly_convertible<DataHolder::Ptr,ISerializable::Ptr>();
//	implicitly_convertible<ISerializable::Ptr,DataHolder::Ptr>();
	implicitly_convertible<DataHolder::Ptr,ElementBase::Ptr>();


	class_<DataHolder, DataHolder::Ptr, bases<ElementBase>>("DataHolder")
			.def("asPointCloud", &DataHolder::asPointCloud)
//			.def("getData", &DataHolder::getData)
			.def("getAllOserations", &DataHolder::getAllObservations)
			.def("newKeypoint", &DataHolder::newKeypoint)
			.def("appendKeypoint", &DataHolder::appendKeypoint)
			.def("initFromCloud", &DataHolder::initFromCloud)
			.def("getTotalNumberOfObservations", &DataHolder::getTotalNumberOfObservations)
			.def("ereaseInvalidObservations", &DataHolder::ereaseInvalidObservations)
			.def("getValidKeypoints", &DataHolder::getValidKeypoints)
			.def("getKeypointsOnMaterial", &DataHolder::getKeypointsOnMaterial)
			.def("getUniqueMaterials", &DataHolder::getUniqueMaterials)
			.def("getDataSources", &DataHolder::getDataSources)
;


	class_<CloudDataSourceOnDisk, CloudDataSourceOnDisk::Ptr, bases<ElementBase>> ("CloudDataSourceOnDisk")
			.def("exists", &CloudDataSourceOnDisk::exists)
			.def("getExtension", &CloudDataSourceOnDisk::getExtension)
			.def("getFilename", &CloudDataSourceOnDisk::getFilename)
			.def("setFilename", &CloudDataSourceOnDisk::setFilename)
//			.def("load", &CloudDataSourceOnDisk::load)
			.def("load2", &CloudDataSourceOnDisk::load2)
			;

//	wrap_std_map<std::string, size_t> ("StdMapStringToSize");

    class_<FieldLabel> ("FieldLabel")
            .def_readwrite("field_name", &FieldLabel::field_name_)
            .def_readwrite("dimensionality", &FieldLabel::dimensionality_)
            ;


    class_<LabelsContainer> ("LabelsContainer")
            .def("hasField", &LabelsContainer::hasField)
            .def("getLabelByName", &LabelsContainer::getLabelByName)
            .def("hasLabel", &LabelsContainer::hasLabel)
            .def("push_back", &LabelsContainer::push_back)
            .def("size", &LabelsContainer::size)
//            .def("getLabel", &LabelsContainer::getLabel)
//            .def("getLabels", &LabelsContainer::getLabels)
            ;

	class_<NewSpcPointCloud, NewSpcPointCloud::Ptr, bases<ElementBase>> ("NewSpcPointCloud")
			.def("getData",  &NewSpcPointCloud::getData)
			.def("getNumberOfPoints",  &NewSpcPointCloud::getNumberOfPoints)
			.def("addNewField", &NewSpcPointCloud::addNewField)
			.def("getFieldByName", static_cast<NewSpcPointCloud::BlockT (NewSpcPointCloud::*)(const std::string &)  >  (&NewSpcPointCloud::getFieldByName ), getFieldByName())
//			.def("getFieldByName", static_cast<NewSpcPointCloud::ConstBlockT (NewSpcPointCloud::*)(const std::string &) const >  (&NewSpcPointCloud::getFieldByName ), getFieldByName())
			.def("getCentroid", &NewSpcPointCloud::getCentroid)
			.def("hasField", &NewSpcPointCloud::hasField)
            .def("filterOutNans", &NewSpcPointCloud::filterOutNans)
            .def("conservativeResize", &NewSpcPointCloud::conservativeResize)
            .def("getSearcher", &NewSpcPointCloud::getSearcher)
            .def("updateSearcher", &NewSpcPointCloud::updateSearcher)
            .def("resetSearcher", &NewSpcPointCloud::resetSearcher)
            .def("__getitem__", &NewSpcPointCloud::getFieldAsMatrixByName)

            .def("getSensor", &NewSpcPointCloud::getSensor)




//			.def_read("fields", &NewSpcPointCloud::fields_)
            .def("getLabels", &NewSpcPointCloud::getLabels)
//			.def_read("fields", &NewSpcPointCloud::field_to_col_)
			;

	class_<KeyPoint, KeyPoint::Ptr>("Keypoint")
			.def_readwrite("fitting_plane", &KeyPoint::fitting_plane)
			.def_readwrite("original_position", &KeyPoint::original_position)
			.def_readwrite("post_position", &KeyPoint::post_position)
			.def_readwrite("eigen_ratio", &KeyPoint::eigen_ratio)
			.def_readwrite("lambdas", &KeyPoint::lambdas)
			.def_readwrite("observations", &KeyPoint::observations)
			.def_readwrite("cumulative_set", &KeyPoint::cumulative_set)
			.def_readwrite("material_id", &KeyPoint::material_id)
			.def_readwrite("s1", &KeyPoint::s1)
			.def_readwrite("s2", &KeyPoint::s2)
			.def_readwrite("center_to_new_center", &KeyPoint::center_to_new_center)
			;


	class_<Observation, Observation::Ptr> ("Observation")
			.def("getCloud", &Observation::getCloud)
			.def("hasValidDistance", &Observation::hasValidDistance)
			.def("hasValidAngle", &Observation::hasValidAngle)

			.def("hasValidIntensity", &Observation::hasValidIntensity)

			.def("getAsEigenPoint", &Observation::getAsEigenPoint)

			.def("isValid", &Observation::isValid)

			.def_readwrite("n_neighbors_intensity", &Observation::n_neighbors_intensity)
			.def_readwrite("distance", &Observation::distance)
			.def_readwrite("angle", &Observation::angle)
			.def_readwrite("intensity", &Observation::intensity)
			.def_readwrite("intensity_std", &Observation::intensity_std)
			.def_readwrite("intensity_corrected", &Observation::intensity_corrected)
//			.def_readwrite("sensor_position", &Observation::sensor_position)
			.def_readwrite("extract_for_normal", &Observation::extract_for_normal_)




		;




}





