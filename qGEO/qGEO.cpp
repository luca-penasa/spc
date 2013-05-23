#include <qGEO.h>


#include <ccPointCloud.h>


#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <ComputeTimeSeries.h>
#include <ComputeStratigraphicPosition.h>
#include <test.h>


qGEO::qGEO()
	: m_toolbar(0)
	, m_menu(0)
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
        addFilter( new ComputeStratigraphicPosition() );
        addFilter( new ComputeTimeSeries() );
//        addFilter( new Test());

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
