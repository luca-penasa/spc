#ifndef CCMYCCHOBJECT_H
#define CCMYCCHOBJECT_H

#include <dialogs/ccProperties.h>


#define MY_CC_OBJECT_BIT                    0x1000000
#define MY_CC_SINGLE_PLANE_MODEL_BIT        0x2000000
#define MY_CC_ORIENTATION_BIT               0x4000000
#define MY_CC_STRAT_MODEL_BIT               0x8000000

enum MY_CC_CLASS_ENUM {
    MY_CC_OBJECT                 = MY_CC_OBJECT_BIT,
    MY_CC_ORIENTATION            = MY_CC_OBJECT_BIT | MY_CC_ORIENTATION_BIT,
    MY_CC_STRAT_MODEL            = MY_CC_OBJECT_BIT | MY_CC_STRAT_MODEL_BIT,
    MY_CC_SINGLE_PLANE_MODEL     = MY_CC_OBJECT_BIT | MY_CC_STRAT_MODEL_BIT | MY_CC_SINGLE_PLANE_MODEL_BIT
};


class ccMyHObject
{
public:
    ccMyHObject ();

    virtual void populateTreeView(ccProperties * prop_widget) = 0;
};

#endif // CCMYCCHOBJECT_H
