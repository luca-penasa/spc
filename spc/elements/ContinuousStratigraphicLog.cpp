#include "ContinuousStratigraphicLog.h"

namespace spc
{


DtiClassType ContinuousStratigraphicLog::Type("ContinuousStratigraphicLog", &StratigraphicPositionableElement::Type);


ContinuousStratigraphicLog::ContinuousStratigraphicLog()
{
    positionable_type_ = EXTENDED;
}

void ContinuousStratigraphicLog::setUserStratigraphicPosition(const float &_arg)
{
    ts_.setXStart(_arg);

}

float ContinuousStratigraphicLog::getUserStratigraphicPosition() const
{
    return ts_.getXStart();
}

}
