#include "DtiClass.h"



DtiClassType::DtiClassType( const std::string name,  DtiClassType * parent)
{
    class_name_ = name;
    parent_ = parent;
}

DtiClassType::~DtiClassType()
{
}



