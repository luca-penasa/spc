/** we register here ALL the serilizable types, so we can add/remove archive types easily
 */

#include <cereal/cereal.hpp>

//supported archives in cereal:
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>
//#include <cereal/archives/portable_binary.hpp>

// some types that cereal need to know how to serialize
#include <spc/common/cereal_types.hpp>

// registering polymorphic stuff
#include <cereal/types/polymorphic.hpp>

#include <spc/elements/attitude.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::Attitude, "Attitude")

#include <spc/elements/cylinder.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::Cylinder, "Cylinder")

#include <spc/elements/ModificableElement.h>
//CEREAL_REGISTER_TYPE_WITH_NAME(spc::ModificableElement, "ModificableElement")

#include <spc/elements/movable_element.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::PositionableElement,"PositionableElement")

#include <spc/elements/normal3d.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::spcNormal3D,"Normal3D")

#include <spc/elements/plane.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::Plane, "Plane")

#include <spc/elements/Sample.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::Sample, "Sample")

#include <spc/elements/SerializableObject.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::spcSerializableObject, "SerializableObject")

#include <spc/elements/spcObject.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::spcObject, "spcObject")

#include <spc/elements/UniversalUniqueID.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::UniversalUniqueID, "UniversalUniqueID")

#include <spc/elements/UniversalUniqueObject.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::UniversalUniqueObject, "UniversalUniqueObject")


#include <spc/tables/UnorderedDataTable.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::UnorderedDataRecord, "UnorderedDataRecord")
CEREAL_REGISTER_TYPE_WITH_NAME(spc::Property, "Property")

#include <spc/time_series/equally_spaced_time_series.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::EquallySpacedTimeSeries<float>, "EquallySpacedTimeSeries")

#include <spc/time_series/sparse_time_series.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::SparseTimeSeries<float>, "SparseTimeSeries")


