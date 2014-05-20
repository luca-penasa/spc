#pragma once
#ifndef SAMPLE_H
#define SAMPLE_H

#include <spc/elements/movable_element.h>
#include <spc/elements/UniversalUniqueObject.h>

#include <spc/common/macros.h>

namespace spc
{

class Sample: public PositionableElement, public UniversalUniqueObject
{
    spcTypedefSmartPointersMacro(Sample)
public:
    Sample()
    {
    }

    spcSetMacro(StratigraphicPosition, stratigraphic_position_, float)
    spcGetMacro(StratigraphicPosition, stratigraphic_position_, float)


protected:
    float stratigraphic_position_;
};

} //end nspace

#endif // SAMPLE_H
