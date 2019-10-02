#pragma once
#ifndef POLYLINE_HPP
#define POLYLINE_HPP

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




}
#endif 
