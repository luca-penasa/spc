#ifndef SPC_SALVABLE_OBJECT_H
#define SPC_SALVABLE_OBJECT_H

//#include <boost/shared_ptr.hpp>
#include <spc/common/macros.h>

//#include <cereal/cereal.hpp>
#include <Eigen/Dense>
#include <fstream>

#include <iostream>

#include <ostream>



namespace spc
{


class spcSerializableObject
{
public:
spcTypedefSmartPointersMacro(spcSerializableObject)


public:
    spcSerializableObject();

    virtual bool isSPCSerializable() {return true;}

    virtual bool canBeSavedAsAscii() const
    {
        return false;
    }

    template<class Archive>
    void serialize(Archive & archive)
    {
        std::cout << "called serialization "<< std::endl;
    }





};









}//end nspace



#endif // SALVABLE_OBJECT_H
