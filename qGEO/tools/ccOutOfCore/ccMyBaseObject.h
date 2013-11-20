#ifndef CCMYBASEOBJECT_H
#define CCMYBASEOBJECT_H

#include <ccOutOfCore/ccEditableHObject.h>
#include <ccHObject.h>
#include <dialogs/ccTimeSeriesGeneratorEditorDlg.h>

#include <spc/elements/element_base.h>

class ccMyBaseObject: public ccEditableHObject, public ccHObject, virtual public spc::spcCommon
{
public:
    ///
    /// \brief ccMyBaseObject def constructor
    ///
    ccMyBaseObject();


};

#endif // CCMYBASEOBJECT_H
