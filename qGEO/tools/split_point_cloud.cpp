#include "split_point_cloud.h"

#include <spc/io/pointcloud2_reader.h>
#include <spc/methods/cloud_slicer.h>



SplitPointCloud::SplitPointCloud( ccPluginInterface * parent_plugin): BaseFilter(FilterDescription(   "Split Point Clouds",
                                                                   "Split the cloud in slices using a scalar field",
                                                                   "Create new clouds splittig the selected one using a scalar field",
                                                                   ":/toolbar/icons/split_cloud.png") , parent_plugin)
{
}


int
SplitPointCloud::openInputDialog()
{
    if (!m_dialog)
        m_dialog = new SplitPointCloudDlg(0);
    else
        m_dialog->comboScalar->clear();

    ccPointCloud * cloud = getSelectedEntityAsCCPointCloud();

    m_dialog->comboScalar->addItemsFromFieldsCloud(cloud);


    return m_dialog->exec() ? 1 : 0;
}


int
SplitPointCloud::compute()
{
    ccPointCloud * cloud = getSelectedEntityAsCCPointCloud();
    if (!cloud)
        return -1; //just to be sure!


    int sf_id = m_dialog->comboScalar->currentIndex();
    ComboItemDescriptor sf_desc = m_dialog->comboScalar->itemData(sf_id).value<ComboItemDescriptor>();
    std::string sf_name = sf_desc.name.toStdString();


    spc::PointCloud2Reader * reader = new spc::PointCloud2Reader(cloud);

    float width = m_dialog->doubleSpinBox_width->value();
    float step = m_dialog->doubleSpinBox_step->value();
    int min_points = m_dialog->spinBox_min_points->value();

    spc::CloudSerializedSlicerOnField splitter;
    splitter.setInputReader(reader);
    splitter.setFieldName(sf_name);
    splitter.setSliceStep(step);
    splitter.setSliceWidth(width);
    splitter.compute();

    auto ids = splitter.getOutputIndices();

    QString container_name = QString("splits_w:") + QString::number(width, 'g', 2) + QString("_s:") + QString::number(step, 'g', 2);
    ccHObject * container = new ccHObject(container_name);



    if (ids.size() == 0)
    {
        return -1;
    }

    for (auto id : ids)
    {
        if (id.size() > min_points)
        {
            CCLib::ReferenceCloud * selected = new CCLib::ReferenceCloud (cloud);

            for (int i =0; i < id.size() ; ++i)
                selected->addPointIndex(id.at(i));


            ccPointCloud * new_cloud =  ccPointCloud::From(selected);
            container->addChild(new_cloud);
        }
    }

    if (container->getChildrenNumber() == 0)
        return -1;

    this->newEntity(container);
    return 1;
}
