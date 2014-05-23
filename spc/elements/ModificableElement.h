#ifndef MODIFICABLEELEMENT_H
#define MODIFICABLEELEMENT_H

#include <spc/common/macros.h>
#include <cereal/cereal.hpp>
#include <pcl/console/print.h>

namespace spc
{

class ModificableElement
{

    SPC_OBJECT(ModificableElement)
public:
    ModificableElement() : modified_(false)
    {}

    // this may be useful in a future
    virtual void modified()
    {
        modified_ = true;
    }

    virtual void update()
    {
        // do stuff in subclasses
        pcl::console::print_debug("Called update in base method, update() not implemented in %s", this->getClassName().c_str());
        modified_ = false;
    }
private:
    friend class cereal::access;

    template <class Archive>
    void serialize( Archive & ar)
    {
        //nothing for now
    }

protected:
    bool modified_;

};


}
#endif // MODIFICABLEELEMENT_H
