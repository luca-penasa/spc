#include "qGEO.h"


#include <ccPointCloud.h>
#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <ComputeTimeSeries.h>
#include <ComputeStratigraphicPosition.h>
#include <FitAttitude.h>
#include <AttitudeToModel.h>
#include <EvaluateStratigraphicPosition.h>
#include <Properties.h>
#include <define2dselection.h>
#include <SetUpNewSeries.h>
#include <split_point_cloud.h>
#include <plot_2d.h>
#include <test.h>


qGEO::qGEO(): m_menu(0)
{
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
        addFilter( new ComputeStratigraphicPosition(this) );
        addFilter( new ComputeTimeSeries(this));
        addFilter(new SplitPointCloud(this));

        addFilter(new EvaluateStratigraphicPosition(this));
        addFilter(new Plot2D(this));
        addFilter(new Properties(this));
        addFilter(new Define2DSelection(this));
        addFilter(new SetUpNewSeries(this));
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

ccCurvePlotterDlg * qGEO::getCurrentPlotter()
{
    Plot2D * example_class = new Plot2D; // this is only an instance for testing
                                         // the typeid in the for...
    //cycle on all enabled filters:
    for (auto  f: m_filters)
    {
        if (typeid(*f) == typeid(*example_class))
        {
            //do a cast
            Plot2D * plot = dynamic_cast<Plot2D *>( f );
            return plot->getPlot();
        }
    }
}

void qGEO::onNewSelection(const ccHObject::Container& selectedEntities)
{
    for (unsigned i=0;i<m_filters.size();++i)
        m_filters[i]->updateSelectedEntities(selectedEntities);
}

QIcon qGEO::getIcon() const
{
    return QIcon(QString::fromUtf8(":/toolbar/qGEO.png"));
}

//plugin export
Q_EXPORT_PLUGIN2(qGEO,qGEO);
