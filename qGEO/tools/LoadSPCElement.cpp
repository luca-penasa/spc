#include "LoadSPCElement.h"


LoadSPCElement::LoadSPCElement(ccPluginInterface * parent_plugin): BaseFilter(FilterDescription(   "Load SPC elemnts",
                                                                                                   "Load SPC elemnts",
                                                                                                   "Load SPC elemnts",
                                                                                                   ":/toolbar/icons/load.png")
                                                                              , parent_plugin)
{

    setShowProgressBar(false);

}

int LoadSPCElement::openOutputDialog()
{

    if (m_filename.isEmpty())
        return 1;

    spc::spcElementSerializer serializer;
    spc::spcSerializableObject::Ptr all_objects = spc::spcSerializableObject::Ptr(new spc::spcSerializableObject);

    bool status;
    status = serializer.load(m_filename.toStdString(), all_objects);

    if (!status)
    {
        ccLog::Error("Cannot load requested archive. Sure about the format?");
        return -1;
    }

    spc::spcSerializableContainer container = *boost::shared_static_cast<spc::spcSerializableContainer>(all_objects);

    for (int i = 0; i < container.getNumberOfElements(); ++i)
    {

        spc::spcSerializableObject * obj = container.at(i);

        ccHObject * HObject = dynamic_cast<ccHObject *> (obj);

        if (!HObject)
            continue; // just to be sure!

        newEntity(HObject);

    }

    return 1;
}
