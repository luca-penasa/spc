#include "qGEO.h"


#include <ccPointCloud.h>
#include <qPCL/PclUtils/filters/BaseFilter.h>
//#include <ComputeTimeSeries.h>
#include <ComputeStratigraphicPosition.h>
#include <FitAttitude.h>
#include <AttitudeToModel.h>
#include <Edit.h>
#include <SaveSPCElement.h>
#include <LoadSPCElement.h>

#include <EvaluateStratigraphicPosition.h>
#include <Properties.h>
#include <CloudToPlanarSelection.h>
#include <SetUpNewSeries.h>
#include <split_point_cloud.h>
#include <OpenPlotsDialog.h>
#include <test.h>

#include <PlotterDlg.h>


static qGEO * qgeo_instance = 0;

qGEO::qGEO()
{
    qgeo_instance = this;
}

qGEO::~qGEO()
{
	while (!m_filters.empty())
	{
		delete m_filters.back();
		m_filters.pop_back();
	}
}

void qGEO::handleNewEntity(ccHObject* entity)
{
	assert(entity && m_app);
    entity->setSelected(true);
	m_app->addToDB(entity);


}

void qGEO::handleEntityChange(ccHObject* entity)
{
	assert(entity && m_app);
	entity->prepareDisplayForRefresh_recursive();
	m_app->refreshAll();
	m_app->updateUI();
}

void qGEO::handleErrorMessage(QString message)
{
	assert(m_app);

	m_app->dispToConsole(qPrintable(message),ccMainAppInterface::ERR_CONSOLE_MESSAGE);
}

void qGEO::getActions(QActionGroup& group)
{
    if (m_filters.empty())
    {
        //ADD FILTERS
        addFilter(new FitAttitude(this));
        addFilter(new AttitudeToModel(this));
        addFilter(new Edit(this));


        addFilter(new OpenPlotsDialog(this));
//        m_plotter = openPlot->getPlotterDlg();

        addFilter(new SetUpNewSeries(this));
        addFilter(new SaveSPCElement(this));
        addFilter(new LoadSPCElement(this));


//        addFilter( new ComputeStratigraphicPosition(this) );
//        addFilter( new ComputeTimeSeries(this));
//        addFilter(new SplitPointCloud(this));

//        addFilter(new EvaluateStratigraphicPosition(this));
//        addFilter(new Properties(this));
        addFilter(new CloudToPlanarSelection(this));

    }

    for (std::vector<BaseFilter*>::const_iterator it = m_filters.begin(); it != m_filters.end(); ++it)
        group.addAction((*it)->getAction());
}

int qGEO::addFilter(BaseFilter * filter)
{
    assert(filter);
    filter->setMainAppInterface(m_app);

    QAction *action = filter->getAction();
    if (!action)
        return 0;

    //filter already inserted?
    if (std::find(m_filters.begin(),m_filters.end(),filter) != m_filters.end())
        return 0;

    m_filters.push_back(filter);

    //connect signals
    connect(filter, SIGNAL(newEntity(ccHObject*)),          this,   SLOT(handleNewEntity(ccHObject*)));
    connect(filter, SIGNAL(entityHasChanged(ccHObject*)),   this,   SLOT(handleEntityChange(ccHObject*)));
    connect(filter, SIGNAL(newErrorMessage(QString)),       this,   SLOT(handleErrorMessage(QString)));

    return 1;
}



ccHObject::Container qGEO::getSelected() const
{
    return m_selected;
}

ccHObject::Container qGEO::getSelectedThatHaveMetaData(const QString key) const
{
    ccHObject::Container sel = getSelected();
    ccHObject::Container new_sel;

    for (ccHObject * obj: sel)
    {
        if (obj->hasMetaData(key))
            new_sel.push_back(obj);

    }

    return new_sel;

}

ccHObject::Container qGEO::getSelectedThatAre(CC_CLASS_ENUM ThisType) const
{
    ccHObject::Container sel = getSelected(); // all selected  

    ccHObject::Container out = qGEO::filterObjectsByType(sel, ThisType);

    return out;

}

ccHObject::Container qGEO::getSelectedKindOf(CC_CLASS_ENUM ThisType)
{
    ccHObject::Container sel = getSelected(); // all selected

    ccHObject::Container out = qGEO::filterObjectsByKind(sel, ThisType);

    return out;

}

ccHObject::Container qGEO::filterObjectsByType(const ccHObject::Container &in, const CC_CLASS_ENUM ThisType)
{
    if (in.empty())
        return in;

    ccHObject::Container out;
    for (ccHObject * obj: in)
    {
        if (obj->isA(ThisType))
        {
            out.push_back(obj);
        }
    }

    return out;
}

ccHObject::Container qGEO::filterObjectsByKind(const ccHObject::Container &in, const CC_CLASS_ENUM ThisType)
{
    if (in.empty())
        return in;

    ccHObject::Container out;
    for (ccHObject * obj: in)
    {
        if (obj->isKindOf(ThisType))
        {
            out.push_back(obj);
        }
    }

    return out;
}

ccHObject::Container qGEO::getAllObjectsInTreeThatHaveMetaData( const QString key)
{

    ccHObject::Container all = this->getAllObjectsInTree();

    return qGEO::filterObjectsByMetaData(all, key);
}

ccHObject::Container qGEO::getAllObjectsInTreeThatAre(CC_CLASS_ENUM ThisType)
{
    ccHObject::Container all = getAllObjectsInTree();
    std::cout << "in tree " <<  all.size() << std::endl;
    return qGEO::filterObjectsByType(all, ThisType);
}

ccHObject::Container qGEO::getAllChildren(ccHObject *object)
{
    int n = object->getChildrenNumber();
    ccHObject::Container cont;
    for (int i = 0; i < n; ++i)
    {
        cont.push_back(object->getChild(i));
    }
    return cont;
}

ccHObject::Container qGEO::filterObjectsByMetaData(const ccHObject::Container &in, const QString key)
{
    ccHObject::Container out;
    for (ccHObject * obj: in)
    {
        if (obj->hasMetaData(key))
        {
            out.push_back(obj);
        }
    }

    return out;
}

ccHObject::Container qGEO::getAllObjectsInTree()
{
//    ccHObject::Container cont;

    ccHObject* dbroot = this->getMainAppInterface()->dbRootObject();
    ccHObject::Container tovisit;
    tovisit.push_back(dbroot);

    ccHObject::Container out;

    while (!tovisit.empty())
    {
        ccHObject  * last = tovisit.back();
        tovisit.pop_back();

        out.push_back(last);

        ccHObject::Container sons = getAllChildren(last);

        for (ccHObject * obj: sons)
            tovisit.push_back(obj);
    }
    return out;
}


void qGEO::onNewSelection(const ccHObject::Container& selectedEntities)
{
    for (unsigned i=0;i<m_filters.size();++i)
        m_filters[i]->updateSelectedEntities(selectedEntities);

    m_selected = selectedEntities;

    emit selectionChanged(m_selected);
}

QIcon qGEO::getIcon() const
{
    return QIcon(QString::fromUtf8(":/toolbar/qGEO.png"));
}

qGEO *qGEO::theInstance()
{
    return qgeo_instance;
}

QMainWindow *qGEO::getMainWindow()
{
    return getMainAppInterface()->getMainWindow();
}

PlotterDlg *qGEO::getPlotterDlg()
{
    for (BaseFilter * f: m_filters)
    {
//        OpenPlotsDialog * open_plot_filter = dynamic_cast<OpenPlotsDialog *>(f);
        if (typeid(*f) ==typeid(OpenPlotsDialog))
        {
            std::cout << "found!"<<std::endl;
            return static_cast<OpenPlotsDialog *> (f)->getPlotterDlg();
        }

    }
    return 0;
}

//plugin export
Q_EXPORT_PLUGIN2
(qGEO,qGEO)
