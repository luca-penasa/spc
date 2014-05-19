#ifndef ELEMENT_IO_H
#define ELEMENT_IO_H

#include <Eigen/Dense>
#include <fstream>

#include <spc/elements/salvable_object.h>
#include <cereal/archives/xml.hpp>

#include <spc/common/eigen_serialization.hpp>

#include <spc/elements/movable_element.h>


namespace spc {

class ElementsIO
{

public:
    ElementsIO()
    {

    }

    void serialize(const PositionableElement & element, const std::string filename)
    {

        std::ofstream os(filename);
        cereal::XMLOutputArchive archive(os);

        archive(element);
    }

};


} //end nspace

#endif // ELEMENT_IO_H
