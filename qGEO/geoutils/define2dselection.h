#ifndef DEFINE2DSELECTION_H
#define DEFINE2DSELECTION_H


#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <dialogs/ccProperties.h>
#include <ccGraphicalSegmentationTool.h>


class Define2DSelection: public BaseFilter
{
Q_OBJECT

public:
    Define2DSelection(ccPluginInterface * parent_plugin = 0);

protected:
   virtual int compute();

    virtual int openInputDialog();

private:
    ccGraphicalSegmentationTool * m_segtool;

public slots:
    void called();


};

#endif // DEFINE2DSELECTION_H
