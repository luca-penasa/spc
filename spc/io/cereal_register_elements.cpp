/** we register here ALL the serilizable types, so we can add/remove archive
 * types easily
 */

#include <cereal/cereal.hpp>

// supported archives in cereal:
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>
//#include <cereal/archives/portable_binary.hpp>

// some types that cereal need to know how to serialize
#include <spc/io/cereal_types.hpp>

// registering polymorphic stuff
#include <cereal/types/polymorphic.hpp>

#include <spc/elements/Attitude.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::Attitude, "Attitude")

#include <spc/elements/Cylinder.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::Cylinder, "Cylinder")

//#include <spc/elements/ModificableElement.h>
// CEREAL_REGISTER_TYPE_WITH_NAME(spc::ModificableElement, "ModificableElement")

#include <spc/elements/MovableElement.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::MovableElement, "MovableElement")

#include <spc/elements/Normal3D.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::Normal3D, "Normal3D")

#include <spc/elements/Plane.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::Plane, "Plane")

#include <spc/elements/Sample.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::Sample, "Sample")

//#include <spc/elements/SerializableObject.h>
// CEREAL_REGISTER_TYPE_WITH_NAME(spc::spcSerializableObject,
// "SerializableObject")

#include <spc/elements/ElementBase.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::ElementBase, "ElementBase")

//#include <spc/elements/UniversalUniqueID.h>
// CEREAL_REGISTER_TYPE_WITH_NAME(spc::UniversalUniqueID, "UniversalUniqueID")

#include <spc/elements/VariantPropertiesRecord.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::VariantPropertiesRecord,
                               "VariantPropertiesRecord")

#include <spc/elements/VariantProperty.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::VariantProperty, "VariantProperty")

#include <spc/elements/VariableScalarFieldBase.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::VariableScalarFieldBase,
                               "VariableScalarFieldBase")

#include <spc/elements/StratigraphicModelBase.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::StratigraphicModelBase,
                               "StratigraphicModelBase")

#include <spc/elements/StratigraphicModelSingleAttitude.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::StratigraphicModelSingleAttitude,
                               "StratigraphicModelSingleAttitude")

#include <spc/elements/TimeSeriesBase.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::TimeSeriesBase<float>, "TimeSeriesBase")

#include <spc/elements/TimeSeriesEquallySpaced.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::TimeSeriesEquallySpaced<float>,
                               "TimeSeriesEquallySpaced")

#include <spc/elements/TimeSeriesSparse.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::TimeSeriesSparse<float>, "TimeSeriesSparse")

#include <spc/elements/SamplesDB.h>
CEREAL_REGISTER_TYPE_WITH_NAME(spc::SamplesDB, "SamplesDB")


