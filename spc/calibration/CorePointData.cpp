#include <spc/calibration/CorePointData.h>


namespace spc
{
std::ostream &operator<<(std::ostream &os, const spc::CorePointData &obj)
{
    // write obj to stream
    spcForEachMacro (CorePointData::PairType iter, obj.getDB())
    {
        os << iter.first << ": ";

        if (iter.second.type() == typeid(std::vector<int>))
        {
            std::vector<int>  ids = boost::any_cast<std::vector<int> > (iter.second);

            spcForEachMacro (int i, ids)
                    os << i << " " ;

            os << std::endl;
        }

        if (iter.second.type() == typeid(std::vector<float>))
        {
            std::vector<float>  ids = boost::any_cast<std::vector<float> > (iter.second);

            spcForEachMacro (float i, ids)
                    os << i << " " ;

            os << std::endl;
        }

        if (iter.second.type() == typeid(float))
        {
            float  data = boost::any_cast<float> (iter.second);
            os << data << std::endl;
        }

        if (iter.second.type() == typeid(int))
        {
            int  data = boost::any_cast<int> (iter.second);
            os << data << std::endl;
        }

        if (iter.second.type() == typeid(size_t) )
        {
            size_t  data = boost::any_cast<size_t> (iter.second);
            os << data << std::endl;
        }

        if (iter.second.type() == typeid(std::string) )
        {
            std::string  data = boost::any_cast<std::string> (iter.second);
            os << data << std::endl;
        }

        if (iter.second.type() == typeid(Eigen::Vector3f))
        {
            Eigen::Vector3f  data = boost::any_cast<Eigen::Vector3f> (iter.second);
            os << data.transpose() << std::endl;
        }
    }
    return os;
}




}//end nspace
