#ifndef SPC_SALVABLE_OBJECT_H
#define SPC_SALVABLE_OBJECT_H

#include <boost/shared_ptr.hpp>

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
    typedef boost::shared_ptr<spcSerializableObject> Ptr;
    typedef boost::shared_ptr<const spcSerializableObject> ConstPtr;


public:
    spcSerializableObject();

    virtual bool isSPCSerializable() {return true;}


    template<class Archive>
    void serialize(Archive & archive)
    {
        std::cout << "called serialization "<< std::endl;
    }





};









}//end nspace



#endif // SALVABLE_OBJECT_H
