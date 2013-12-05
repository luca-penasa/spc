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
        stream.precision (6); //6digits
        stream.imbue (std::locale::classic ());

        // get all qgeo-valid objects that are selected
        ccHObject::Container all = qGEO::theInstance()->getSelectedThatHaveMetaData("[qGEO]");

//        for (ccHObject * obj: all)
//        {

        ccHObject * obj = all.at(0);

            spc::spcElementBase * common = dynamic_cast<spc::spcElementBase *> (obj);

            if (!common)
                std::cout << "cast not valid!" << std::endl;

            std::ofstream ofs("/home/luca/filename.xml");
            boost::archive::xml_oarchive oa(ofs);
            oa << BOOST_SERIALIZATION_NVP(*common);

            //save the class type:

//            stream << "#" << common->getSPCClassName() << std::endl;
//            common->toStringStreamMeOnly(stream);/

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
