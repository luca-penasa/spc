#ifndef OPENINCOMPOSER_H
#define OPENINCOMPOSER_H

#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <qGEO/qGEO.h>
#include <dialogs/Composer.h>
#include <spc/elements/element_base.h>


#include <ccOutOfCore/ccMyBaseObject.h>
#include<ccOutOfCore/ccAttitude.h>
#include <boost/typeof/typeof.hpp>

class SaveSPCElement: public BaseFilter
{
public:
    SaveSPCElement(ccPluginInterface * parent_plugin);

    virtual int compute ()
    {




        return 1;
    }


    virtual int openOutputDialog()
    {
        if (m_filename.isEmpty())
            return 1;


        std::stringstream stream;


//EASILY CONFIGURABLE HERE
//        stream.precision (precision_);
//        stream.imbue (std::locale::classic ());

        // get all qgeo-valid objects that are selected
        ccHObject::Container qua = qGEO::theInstance()->getSelectedThatHaveMetaData("[qGEO]");


        ccHObject * obj = qua.at(0);

        std::cout << obj->isA((CC_HIERARCHY_OBJECT)) << std::endl;



//        ccAttitude * att = static_cast<ccAttitude *> (obj);
//               ccMyBaseObject * base = static_cast<ccMyBaseObject *>(obj);


//        spc::UtilityCommonClass * common = static_cast<spc::UtilityCommonClass *> (base);

//        spc::ElementBase * el = static_cast<spc::ElementBase *> (common);

//        auto * ecco = boost::type(*obj);
//        typedef typename boost::type_of(*obj)  thistype;
//        auto ecco = static_cast< boost::type_of(*obj) * > (obj);

        std::cout << typeid(*obj).name()  << std::endl;

//        std::cout << ( typeid(*obj) == typeid(ccHObject)  )<< std::endl;
       ccMyBaseObject * base = static_cast<ccMyBaseObject *> (obj);

        std::cout << typeid(*base).name()  << std::endl;


        ccAttitude * att = static_cast<ccAttitude * > (base);

        spc::spcElementBase * el = static_cast<spc::spcElementBase *> (att);


        spc::spcCommon * common = dynamic_cast<spc::spcCommon *> (obj);
        spc::spcElementBase * el2 = dynamic_cast<spc::spcElementBase *> (common);



//        spc::Common *common = static_cast<spc::Common *> (base);
//        spc::spcElementBase * el =static_cast<spc::spcElementBase *>(common);

        att->toAsciiMeOnly(stream);
        el->toAsciiMeOnly(stream);
//        common->toAsciiMeOnly(stream);
        el2->toAsciiMeOnly(stream);

        std::cout << stream.str().c_str() << std::endl;



//        el->printSomething();

//        std::cout << el->getSPCClassName() << std::endl;


//        ccMyBaseObject * base = static_cast<ccMyBaseObject *>(obj);


////        spc::UtilityCommonClass * common = dynamic_cast<spc::UtilityCommonClass *>(base);

//        spc::ElementBase * element = static_cast<spc::ElementBase *>( base );

//        if (!element)
//        {
//            std::cout << "casting not valid" << std::endl;
//            return -1;

//        }

//        element->printSomething();
//        std::cout << element->getClassName().c_str() << std::endl;

//        element->ToStreamCompletely(stream);

//        for (ccHObject *obj: qua)
//        {
//            //try to cast
//            ccMyBaseObject * base = static_cast<ccMyBaseObject *>(obj);

//            spc::UtilityCommonClass * common = static_cast<spc::UtilityCommonClass *>(base);

//            spc::ElementBase * element = static_cast<spc::ElementBase *> (common);




//            if (!element)
//                continue;

//            element->printSomething();
//            std::cout << element->getClassName().c_str() << std::endl;

//            element->ToStreamCompletely(stream);

//        }

        std::ofstream file;
        file.open(m_filename.toStdString().c_str());

        file << stream.str();

        file.close(); // and we are fine!


        return 1;
    }

    virtual int openInputDialog()
    {
        m_filename.clear();
        m_filename =  QFileDialog::getSaveFileName();

        return 1;
    }

    virtual int checkSelected()
    {
        return 1;// for now
        ccHObject::Container qua = qGEO::theInstance()->getSelectedThatHaveMetaData("[qGEO]");

        if (qua.size() > 0)
        {
            return 1;
        }
        else
            return -1;
    }


protected:
    QString m_filename;


};

#endif // OPENINCOMPOSER_H
