#include "qtHelper.h"
#include <boost/graph/graph_concepts.hpp>

std::ostream&  operator<<(std::ostream& stream,ComboItemDescriptor ob)
{
    stream<< "name: " <<ob.name.toStdString()<<"  cloud_id: "<<ob.index_in_cloud<<"  combo_id: "<<ob.index_in_combo<< "  type: " << ob.type << '\n';
    return stream;
}


std::vector<float> getComboItemAsStdFloatVector(ComboItemDescriptor desc, const ccPointCloud* cloud)
{
    int n = cloud->size();
    std::vector<float> v;
    v.resize(n);
    v.reserve(n);
    if (desc.type == ComboItemDescriptor::COORDINATE)
    {
        CCVector3 point;
        for (int i = 0; i < n; i++)
        {
            cloud->getPoint(i, point);
            v[i] = point[desc.index_in_cloud];
        }
    }
    else if (desc.type == ComboItemDescriptor::SCALAR)
    {
        CCLib::ScalarField * field = cloud->getScalarField(desc.index_in_cloud);
        for (int i = 0; i < n; i++)
            v[i] = field->getValue(i);

    }

    
    return v;
}




void ScalarFieldsComboBox::addItemsFromFieldsCloud(const ccPointCloud * cloud)
{

    const int n = cloud->getNumberOfScalarFields();
    for (int i = 0; i < n ; i++)
    {
        ComboItemDescriptor desc;
        desc.name = cloud->getScalarFieldName(i);
        desc.index_in_cloud = i;
        desc.type = desc.SCALAR;
        desc.index_in_combo  = this->count() + 1;

        //put in a qVariant
        QVariant data;
        data.setValue(desc);

        this->addItem(desc.name, data);
    }
}

void ScalarFieldsComboBox::addItemsXYZ()
{
    QStringList list;
    list.reserve(3);
    list.append("X Coord");
    list.append("Y Coord");
    list.append("Z Coord");

    for (int i =0; i < list.size(); ++i)
    {
        ComboItemDescriptor desc;
        desc.name = list.at(i);
        desc.index_in_cloud = i;
        desc.index_in_combo = this->count()  + 1;
        desc.type = desc.COORDINATE;

        QVariant data;
        data.setValue(desc);
        
        this->addItem(list.at(i), data);
    }
}

void ScalarFieldsComboBox::addItemsRGB(bool add_also_composite)
{

    QStringList list;
    if (add_also_composite)
        list.reserve(4);
    else
        list.reserve(3);

    list.append("R Color");
    list.append("G Color");
    list.append("B Color");
    if (add_also_composite)
        list.append("RGB Composite");


    for (int i =0; i < list.size(); ++i)
    {
        ComboItemDescriptor desc;
        desc.name = list.at(i);
        desc.index_in_cloud = i;
        desc.index_in_combo = this->count()  + 1;
        desc.type = desc.RGB;

        QVariant data;
        data.setValue(desc);
        
        this->addItem(list.at(i), data);
    }
}


int
ccVector3fromQstring(QString &string, CCVector3 &vector_output, QString separator)
{

    QStringList splitted = string.split(separator);

    //check size
    if (splitted.size() < 3)
        return -1;

    bool *ok = new bool;
    *ok = true;
    for (int i = 0; i < 3; ++i)
    {
        vector_output[i] = splitted.at(i).toFloat(ok);
        if (!ok)
            return -1;
    }

    return 1;


}

bool isLastOfListAnInteger(QStringList list, int &number)
{
    QString last_string = list.at(list.size()-1);
    bool * ok = new bool;
    number = last_string.toInt(ok);
    return *ok;
}

QString suggestIncrementalName(QString name)
{
    int number;
    //try split with a space
    QStringList list = name.split(" ");
    if (isLastOfListAnInteger(list, number));
    {
        QString id = QString().setNum(number + 1);
        return name + QString(" ") + QString(id + 1);
    }

    //try another split
    list = name.split("_");
    if (isLastOfListAnInteger(list, number));
    {
        QString id = QString().setNum(number + 1);
        return name + QString("_") + QString(id + 1);
    }

    //try another split
    list = name.split("-");
    if (isLastOfListAnInteger(list, number));
    {
        QString id = QString().setNum(number + 1);
        return name + QString("-") + QString(id + 1);
    }

    //if we are here no good splits have been found.
    //So simply give a new name
    QString id = QString().setNum(1);
    return name + QString("_") + QString(1);

}



ObjectSelectionComboBox::ObjectSelectionComboBox(QWidget *parent): QComboBox(parent), m_has_none(false), m_old_selected_id(-1)
{
    connect(this, SIGNAL(currentIndexChanged(int)), this, SLOT(updateCurrentSelectionInfo(int)));
}

void ObjectSelectionComboBox::updateObjects(ccHObject::Container &cont)
{
    int old_selected = m_old_selected_id; //first of all!

    if (cont.empty())
        return;
    //clear everything
    this->clear();

    m_container = cont;

    for (ccHObject * obj : cont)
    {
        QString name = obj->getName();
        this->addItem(name);
    }

    if (this->hasNone())
        this->addItem("None"); // it correspond to -1 old id

    int position;
    if (isPresentObjectWithID(old_selected, position))
    {
        // restore it as selected
        this->setCurrentIndex(position);
    }
}

bool ObjectSelectionComboBox::isPresentObjectWithID(const int id, int &position) const
{
    position = 0;
    for (ccHObject * obj: m_container)
    {
        if (obj->getUniqueID() == id)
        {
            return true;
        }
        position +=1;
    }



    return false;
}


ccHObject *ObjectSelectionComboBox::getSelected() const
{
    int id = this->currentIndex();

    if (id >= m_container.size())
        return 0;       

    return m_container.at(id);
}



void ObjectSelectionComboBox::updateCurrentSelectionInfo(int id)
{
    if (id >= m_container.size()) // is a None
        m_old_selected_id = -1;
    else
    {
        ccHObject * obj = m_container.at(id);
        m_old_selected_id = obj->getUniqueID();
    }
}
