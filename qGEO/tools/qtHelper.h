#ifndef Q_GEO_PLUGIN_QTHELPER_H
#define Q_GEO_PLUGIN_QTHELPER_H


#include <QComboBox>
#include <ccPointCloud.h>
#include <iostream>
#include <fstream>

class QString;



//to help keeping track of the selected item and its meaning
struct ComboItemDescriptor
{
    enum allowed_types {SCALAR, COORDINATE, RGB};
    QString name;
    int index_in_cloud;
    int index_in_combo;
    allowed_types type;

    friend std::ostream& operator<<(std::ostream& stream,ComboItemDescriptor ob);


};




std::vector<float> getComboItemAsStdFloatVector(ComboItemDescriptor desc, const ccPointCloud * cloud);


class ScalarFieldsComboBox: public QComboBox
{
    Q_OBJECT

public:
    ScalarFieldsComboBox(QWidget * parent = 0) : QComboBox(parent) {}

    void addItemsFromFieldsCloud(const ccPointCloud * cloud);

    void addItemsXYZ();

    void addItemsRGB(bool add_also_composite=true);


};



class ObjectSelectionComboBox: public QComboBox
{
    Q_OBJECT

public:
    ObjectSelectionComboBox(QWidget * parent =0);

    void updateObjects(ccHObject::Container & cont);

    void setNone(bool has) {m_has_none = has;}

    bool hasNone() const {return m_has_none;}

    bool isPresentObjectWithID(const int id, int &position) const;

    // if none -> void
    ccHObject * getSelected() const;

public slots:
    void updateCurrentSelectionInfo(int id);


private:
    bool m_has_none;

    int m_old_selected_id;

    /// where we keep a copy of the full container, for easy retrieving
    ccHObject::Container m_container;

};


QString suggestIncrementalName(QString name);

bool isLastOfListAnInteger(QStringList list, int &number);


int ccVector3fromQstring(QString &string, CCVector3 &output_vector, QString separator = " ");

Q_DECLARE_METATYPE (ComboItemDescriptor)

#endif //Q_PCL_PLUGIN_QTHELPER_H
