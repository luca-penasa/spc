#include "Fields.h"
namespace spc
{



DtiClassType FieldBase::Type = DtiClassType("FieldBase", &ElementBase::Type);

DtiClassType FieldFloat::Type = DtiClassType("FieldFloat", &FieldBase::Type);
DtiClassType FieldInt::Type = DtiClassType("FieldInt", &FieldBase::Type);
DtiClassType FieldString::Type = DtiClassType("FieldString", &FieldBase::Type);
DtiClassType FieldVector3f::Type
    = DtiClassType("FieldVector3f", &FieldBase::Type);
DtiClassType FieldVectorXf::Type
    = DtiClassType("FieldVectorXf", &FieldBase::Type);
DtiClassType FieldVectorXi::Type
    = DtiClassType("FieldVectorXi", &FieldBase::Type);

DtiClassType FieldVectorStdf::Type
    = DtiClassType("FieldVectorStdf", &FieldBase::Type);

DtiClassType FieldVectorStdi::Type
    = DtiClassType("FieldVectorStdi", &FieldBase::Type);

DtiClassType FieldsManager::Type
    = DtiClassType("FieldsManager", &ElementBase::Type);

// static stuff
std::map
    <FieldBase::SUPPORTED_TYPES, const std::type_info *> FieldBase::id_to_info_{
        { FieldBase::INT, &typeid(int) },
        { FieldBase::FLOAT, &typeid(float) },
        { FieldBase::STRING, &typeid(std::string) },
        { FieldBase::VEC3f, &typeid(Eigen::Vector3f) },
        { FieldBase::VECXf, &typeid(Eigen::VectorXf) },
        { FieldBase::VECXi, &typeid(Eigen::VectorXi) },
        { FieldBase::STDVECf, &typeid(std::vector<float>) },
        { FieldBase::STDVECi, &typeid(std::vector<int>) }
    };

std::map
    <const std::type_info *, FieldBase::SUPPORTED_TYPES> FieldBase::info_to_id_{
        { &typeid(int), FieldBase::INT },
        { &typeid(float), FieldBase::FLOAT },
        { &typeid(std::string), FieldBase::STRING },
        { &typeid(Eigen::Vector3f), FieldBase::VEC3f },
        { &typeid(Eigen::VectorXf), FieldBase::VECXf },
        { &typeid(Eigen::VectorXi), FieldBase::VECXi },
        { &typeid(std::vector<float>), FieldBase::STDVECf },
        { &typeid(std::vector<int>), FieldBase::STDVECi }
    };

} // end nspace
