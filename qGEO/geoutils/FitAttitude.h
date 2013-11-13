#ifndef FITGEOLOGICALORIENTATION_H
#define FITGEOLOGICALORIENTATION_H

#include <qPCL/PclUtils/filters/BaseFilter.h>


class FitAttitude: public BaseFilter
{
public:
    FitAttitude(ccPluginInterface * parent_plugin = 0);


    virtual int compute();

protected:
    virtual int checkSelected();



};

#endif // FITGEOLOGICALORIENTATION_H
