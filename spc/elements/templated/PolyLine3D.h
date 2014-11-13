#ifndef POLYLINE3D_H
#define POLYLINE3D_H

#include <spc/elements/templated/PointSet.h>



namespace spc
{



template<typename ScalarT = float, size_t DIM = 3>
class PolyLine: public PointSet<ScalarT, DIM>
{
public:

    typedef PointSet<ScalarT, DIM> PointSetT;
    typedef PolyLine<ScalarT, DIM> self_t;

    PolyLine() {}

    PolyLine(const PolyLine& set): PointSetT(set)
    {
    }

    PolyLine(const PointSetT &set): PointSetT(set)
    {

    }
};




typedef  PolyLine<float, 3> PolyLine3D;
typedef PolyLine<float, 2> PolyLine2D;


}
#endif // POLYLINE3D_H
