#ifndef LOAD_SPC_ELEMENT_H
#define LOAD_SPC_ELEMENT_H

#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <qGEO/qGEO.h>
#include <dialogs/Composer.h>
#include <spc/elements/element_base.h>


#include <ccOutOfCore/ccMyBaseObject.h>
#include<ccOutOfCore/ccAttitude.h>
#include <boost/typeof/typeof.hpp>

#include <spc/elements/spcSerializableContainer.h>

#include <spc/io/element_io.h>



class LoadSPCElement: public BaseFilter
{
public:
    LoadSPCElement(ccPluginInterface * parent_plugin);

    virtual int compute ()
    {
        return 1;
    }


    virtual int openOutputDialog();

    virtual int openInputDialog()
    {
        m_filename.clear();
        m_filename =  QFileDialog::getOpenFileName(0, tr("Load From XML File"),
                                                   "",
                                                   tr("XML Documents (*.xml)"));
        return 1;
    }

    virtual int checkSelected()
    {
        return 1;
    }


protected:
    QString m_filename;


};

#endif // OPENINCOMPOSER_H
