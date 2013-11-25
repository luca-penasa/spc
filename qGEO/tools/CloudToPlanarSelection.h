#ifndef DEFINE2DSELECTION_H
#define DEFINE2DSELECTION_H


#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <dialogs/ccProperties.h>
#include <ccGraphicalSegmentationTool.h>


class CloudToPlanarSelection: public BaseFilter
{
Q_OBJECT

public:
    CloudToPlanarSelection(ccPluginInterface * parent_plugin = 0);

protected:
   virtual int compute();

    virtual int openInputDialog();

    virtual int checkParameters();

private:
    ccGraphicalSegmentationTool * m_segtool;

//public slots:
//    void called();


};

#endif // DEFINE2DSELECTION_H
