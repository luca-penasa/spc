#ifndef Q_GEO_PLUGIN_HEADER
#define Q_GEO_PLUGIN_HEADER

#include <QObject>
#include <QtGui>
#include <mainwindow.h>

#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <ComputeTimeSeries.h>

#include <ccStdPluginInterface.h>
#include <PlotterDlg.h>


class QToolBar;
class QMenu;

class qGEO;

//qGEO * QGEO_MAIN_PLUGIN(0); // when qGEO will be instantiated it will point to the qGEO instance

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


    virtual QString getName() const { return "qGEO"; }
    virtual QString getDescription() const { return "Geologic stuff for qCC"; }
    virtual QIcon getIcon() const;

    static qGEO *theInstance();

    QString getErrorMessage(int errorCode/*, LANGUAGE lang*/);

    virtual void onNewSelection(const ccHObject::Container& selectedEntities);
    virtual void getActions(QActionGroup& group);

	//! Adds a filter
    int addFilter(BaseFilter* filter);

    //! Return a plotter object for plotting things
    //! for now only one plotter at a time is possible!
    PlotterDlg *getPlotterDlg();

    ///////// ACCESS TO DBTREE ///////////////////////////////
    ccHObject::Container getSelected() const;


    ccHObject::Container getSelectedThatHaveMetaData(const QString key) const;

    ccHObject::Container getSelectedThatAre(CC_CLASS_ENUM ThisType) const;

    ccHObject::Container getAllObjectsInTree();
    ccHObject::Container getAllObjectsInTreeThatHaveMetaData(const QString key );
    ccHObject::Container getAllObjectsInTreeThatAre(CC_CLASS_ENUM ThisType);


    ///////// STATIC FILTERS //////////////////////////////////
    static ccHObject::Container filterObjectsByType(const ccHObject::Container &in, const CC_CLASS_ENUM ThisType);

    static ccHObject::Container filterObjectsByMetaData (const ccHObject::Container &in, const QString key);


signals:
    void selectionChanged(ccHObject::Container & selection);

public slots:
	//! Handles new entity
	void handleNewEntity(ccHObject*);

	//! Handles entity (visual) modification
	void handleEntityChange(ccHObject*);

	//! Handles new error message
	void handleErrorMessage(QString);
	

protected:
	//! Loaded filters
	std::vector<BaseFilter*> m_filters;


    PlotterDlg * m_plotter;


    ccHObject::Container m_selected;

};


#endif//END Q_GEO_PLUGIN_HEADER
