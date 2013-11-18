#ifndef CCMYBASEOBJECT_H
#define CCMYBASEOBJECT_H

#include <ccOutOfCore/ccEditableHObject.h>
#include <ccHObject.h>
#include <dialogs/ccTimeSeriesGeneratorEditorDlg.h>

class ccMyBaseObject: public  ccEditableHObject, public ccHObject
{
public:
    ///
    /// \brief ccMyBaseObject def constructor
    ///
    ccMyBaseObject();

};

#endif // CCMYBASEOBJECT_H
