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
    ObjectSelectionComboBox(QWidget * parent =0): QComboBox(parent), m_has_none(false) {}

    static ObjectSelectionComboBox * fromContainer(ccHObject::Container &cont);

    void addObjects(ccHObject::Container & cont);

    void setHasNone(bool has) {m_has_none = has;}

    bool getHasNone() const {return m_has_none;}

private:
    bool m_has_none;

};


QString suggestIncrementalName(QString name);

bool isLastOfListAnInteger(QStringList list, int &number);


int ccVector3fromQstring(QString &string, CCVector3 &output_vector, QString separator = " ");

Q_DECLARE_METATYPE(ComboItemDescriptor);
#endif //Q_PCL_PLUGIN_QTHELPER_H
