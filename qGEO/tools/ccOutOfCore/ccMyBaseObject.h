#ifndef CCMYBASEOBJECT_H
#define CCMYBASEOBJECT_H

#include <ccOutOfCore/ccEditableHObject.h>
#include <ccHObject.h>
#include <dialogs/ccTimeSeriesGeneratorEditorDlg.h>

#include <spc/elements/element_base.h>
#include <boost/serialization/string.hpp>


/// teach boost how to serialize QString -  using a std::string
namespace boost
{

namespace serialization {

template<class Archive>
inline void load(
        Archive & ar,
        QString& t,
        const unsigned int file_version
        )
{
    std::string tmp;
    ar >> boost::serialization::make_nvp("string", tmp);

    t = QString::fromStdString(tmp);

}

template<class Archive>
inline void save(
        Archive & ar,
        const QString& t,
        const unsigned int file_version
        )
{
    std::string standard = t.toStdString();
    ar << boost::serialization::make_nvp("string", standard);
}



} //end nspace serialization
}//end nspace boost


BOOST_SERIALIZATION_SPLIT_FREE(QString)




class ccMyBaseObject: public ccEditableHObject, public ccHObject, virtual public spc::spcCommon
{
                                 public:
                                 ///
                                 /// \brief ccMyBaseObject def constructor
                                 ///
                                 ccMyBaseObject();

protected:
friend class boost::serialization::access;

template <class Archive>
void serialize(Archive &ar, const unsigned int version)
{
    //        ar & BOOST_SERIALIZATION_NVP(m_scale);
    //        ar & BOOST_SERIALIZATION_NVP(m_width);
    //        ar & boost::serialization::make_nvp("spcAttitude", boost::serialization::base_object<spc::spcAttitude> (*this));
    //        ar & boost::serialization::make_nvp("ccMyBaseObject", boost::serialization::base_object<ccMyBaseObject> (*this));


    ar & BOOST_SERIALIZATION_NVP(m_visible);
    ar & BOOST_SERIALIZATION_NVP(m_colorsDisplayed);
    ar & BOOST_SERIALIZATION_NVP(m_showNameIn3D);
    ar & BOOST_SERIALIZATION_NVP(m_name);

    //        //'visible' state (dataVersion>=20)
    //        if (out.write((const char*)&m_visible,sizeof(bool))<0)
    //            return WriteError();
    //        //'lockedVisibility' state (dataVersion>=20)
    //        if (out.write((const char*)&m_lockedVisibility,sizeof(bool))<0)
    //            return WriteError();
    //        //'colorsDisplayed' state (dataVersion>=20)
    //        if (out.write((const char*)&m_colorsDisplayed,sizeof(bool))<0)
    //            return WriteError();
    //        //'normalsDisplayed' state (dataVersion>=20)
    //        if (out.write((const char*)&m_normalsDisplayed,sizeof(bool))<0)
    //            return WriteError();
    //        //'sfDisplayed' state (dataVersion>=20)
    //        if (out.write((const char*)&m_sfDisplayed,sizeof(bool))<0)
    //            return WriteError();
    //        //'colorIsOverriden' state (dataVersion>=20)
    //        if (out.write((const char*)&m_colorIsOverriden,sizeof(bool))<0)
    //            return WriteError();
    //        if (m_colorIsOverriden)
    //        {
    //            //'tempColor' (dataVersion>=20)
    //            if (out.write((const char*)m_tempColor,sizeof(colorType)*3)<0)
    //                return WriteError();
    //        }
    //        //'glTransEnabled' state (dataVersion>=20)
    //        if (out.write((const char*)&m_glTransEnabled,sizeof(bool))<0)
    //            return WriteError();
    //        if (m_glTransEnabled)
    //            if (!m_glTrans.toFile(out))
    //                return false;

    //        //'showNameIn3D' state (dataVersion>=24)
    //        if (out.write((const char*)&m_showNameIn3D,sizeof(bool))<0)
    //            return WriteError();

    //        return true;

}


};

#endif // CCMYBASEOBJECT_H
