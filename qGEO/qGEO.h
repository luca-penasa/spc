#ifndef Q_GEO_PLUGIN_HEADER
#define Q_GEO_PLUGIN_HEADER

#include <QObject>
#include <QtGui>
#include <mainwindow.h>

#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <ccStdPluginInterface.h>

class PlotterDlg;
class QToolBar;
class QMenu;
class OpenPlotsDialog;

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
//    PlotterDlg *getPlotterDlg();

//    OpenPlotsDialog * getPlotTool();

    QMainWindow * getMainWindow();

    PlotterDlg * getPlotterDlg();



    ///////// ACCESS TO DBTREE ///////////////////////////////
    ccHObject::Container getSelected() const;


    ccHObject::Container getSelectedThatHaveMetaData(const QString key) const;

    ccHObject::Container getSelectedThatAre(CC_CLASS_ENUM ThisType) const;

    ccHObject::Container getAllObjectsInTree();
    ccHObject::Container getAllObjectsInTreeThatHaveMetaData(const QString key );
    ccHObject::Container getAllObjectsInTreeThatAre(CC_CLASS_ENUM ThisType);

    static ccHObject::Container getAllChildren(ccHObject * object);


    ///////// STATIC FILTERS //////////////////////////////////
    static ccHObject::Container filterObjectsByType(const ccHObject::Container &in, const CC_CLASS_ENUM ThisType);

    static ccHObject::Container filterObjectsByMetaData (const ccHObject::Container &in, const QString key);


    ccHObject::Container filterObjectsByKind(const ccHObject::Container &in, const CC_CLASS_ENUM ThisType);

    ccHObject::Container getSelectedKindOf(CC_CLASS_ENUM ThisType);


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

    ccHObject::Container m_selected;

};


#endif//END Q_GEO_PLUGIN_HEADER
