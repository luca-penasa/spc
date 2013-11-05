#ifndef Q_GEO_PLUGIN_HEADER
#define Q_GEO_PLUGIN_HEADER

#include <QObject>
#include <QtGui>
//#include "../ccStdPluginInterface.h"
#include <mainwindow.h>

#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <ComputeTimeSeries.h>

#include <ccStdPluginInterface.h>


class QToolBar;
class QMenu;

//! PCL bridge plugin
class qGEO : public QObject, public ccStdPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(ccStdPluginInterface)

public:
    //! Default constructor
    qGEO();

    //!Destructor
    virtual ~qGEO();

//    //inherited from ccPluginInterface
//    void getDescription(ccPluginDescription& desc);
//	void newSelection(std::vector<ccHObject*>& selectedEntities);
//    QIcon getIcon() const;

    virtual QString getName() const { return "qGEO"; }
    virtual QString getDescription() const { return "Geologic stuff for qCC"; }
    virtual QIcon getIcon() const;

    //inherited from ccStdPluginInterface
//    bool isEnabled(const std::vector<ccHObject*>& selectedEntities);

    QString getErrorMessage(int errorCode/*, LANGUAGE lang*/);


    //inherited from ccStdPluginInterface
    virtual void onNewSelection(const ccHObject::Container& selectedEntities);
    virtual void getActions(QActionGroup& group);
//    int doAction(ccHObject::Container& selectedEntities,

//                 unsigned& uiModificationFlags,
//                 ccProgressDialog* progressCb=NULL,
//                 QWidget* parent=NULL);

	//! Adds a filter
    int addFilter(BaseFilter* filter);

    //! Return a plotter object for plotting things
    //! for now only one plotter at a time is possible!
    ccCurvePlotterDlg * getCurrentPlotter();



public slots:
	//! Handles new entity
	void handleNewEntity(ccHObject*);

	//! Handles entity (visual) modification
	void handleEntityChange(ccHObject*);

	//! Handles new error message
	void handleErrorMessage(QString);
	

protected:

	//! Menu
    QMenu* m_menu;


	//! Loaded filters
	std::vector<BaseFilter*> m_filters;
};


#endif//END Q_GEO_PLUGIN_HEADER
