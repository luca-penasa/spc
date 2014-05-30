#include "spline.h"

// https://github.com/toastedcrumpets/DynamO/blob/master/src/magnet/magnet/math/spline.hpp

double magnet::math::Spline::operator()(double xval)
{
    if (!_valid)
        generate();

    // Special cases when we're outside the range of the spline points
    if (xval <= x(0))
        return lowCalc(xval);
    if (xval >= x(size() - 1))
        return highCalc(xval);

    // Check all intervals except the last one
    for (std::vector<SplineData>::const_iterator iPtr = _data.begin();
         iPtr != _data.end() - 1; ++iPtr)
        if ((xval >= iPtr->x) && (xval <= (iPtr + 1)->x))
            return splineCalc(iPtr, xval);

    return splineCalc(_data.end() - 1, xval);
}

void magnet::math::Spline::generate()
{
    if (size() < 2)
        throw std::runtime_error("Spline requires at least 2 points");

    // If any spline points are at the same x location, we have to
    // just slightly seperate them
    {
        bool testPassed(false);
        while (!testPassed) {
            testPassed = true;
            std::sort(base::begin(), base::end());

            for (auto iPtr = base::begin(); iPtr != base::end() - 1; ++iPtr)
                if (iPtr->first == (iPtr + 1)->first) {
                    if ((iPtr + 1)->first != 0)
                        (iPtr + 1)->first += (iPtr + 1)->first
                                             * std::numeric_limits
                                             <double>::epsilon() * 10;
                    else
                        (iPtr + 1)->first = std::numeric_limits
                                            <double>::epsilon() * 10;
                    testPassed = false;
                    break;
                }
        }
    }

    const size_t e = size() - 1;

    switch (_type) {
    case LINEAR: {
        _data.resize(e);
        for (size_t i(0); i < e; ++i) {
            _data[i].x = x(i);
            _data[i].a = 0;
            _data[i].b = 0;
            _data[i].c = (y(i + 1) - y(i)) / (x(i + 1) - x(i));
            _data[i].d = y(i);
        }
        break;
    }
    case CUBIC: {
        ublas::matrix<double> A(size(), size());
        for (size_t yv(0); yv <= e; ++yv)
            for (size_t xv(0); xv <= e; ++xv)
                A(xv, yv) = 0;

        for (size_t i(1); i < e; ++i) {
            A(i - 1, i) = h(i - 1);
            A(i, i) = 2 * (h(i - 1) + h(i));
            A(i + 1, i) = h(i);
        }

        ublas::vector<double> C(size());
        for (size_t xv(0); xv <= e; ++xv)
            C(xv) = 0;

        for (size_t i(1); i < e; ++i)
            C(i) = 6
                   * ((y(i + 1) - y(i)) / h(i) - (y(i) - y(i - 1)) / h(i - 1));

        // Boundary conditions
        switch (_BCLow) {
        case FIXED_1ST_DERIV_BC:
            C(0) = 6 * ((y(1) - y(0)) / h(0) - _BCLowVal);
            A(0, 0) = 2 * h(0);
            A(1, 0) = h(0);
            break;
        case FIXED_2ND_DERIV_BC:
            C(0) = _BCLowVal;
            A(0, 0) = 1;
            break;
        case PARABOLIC_RUNOUT_BC:
            C(0) = 0;
            A(0, 0) = 1;
            A(1, 0) = -1;
            break;
        }

        switch (_BCHigh) {
        case FIXED_1ST_DERIV_BC:
            C(e) = 6 * (_BCHighVal - (y(e) - y(e - 1)) / h(e - 1));
            A(e, e) = 2 * h(e - 1);
            A(e - 1, e) = h(e - 1);
            break;
        case FIXED_2ND_DERIV_BC:
            C(e) = _BCHighVal;
            A(e, e) = 1;
            break;
        case PARABOLIC_RUNOUT_BC:
            C(e) = 0;
            A(e, e) = 1;
            A(e - 1, e) = -1;
            break;
        }

        ublas::matrix<double> AInv(size(), size());
        InvertMatrix(A, AInv);

        _ddy = ublas::prod(C, AInv);

        _data.resize(size() - 1);
        for (size_t i(0); i < e; ++i) {
            _data[i].x = x(i);
            _data[i].a = (_ddy(i + 1) - _ddy(i)) / (6 * h(i));
            _data[i].b = _ddy(i) / 2;
            _data[i].c = (y(i + 1) - y(i)) / h(i) - _ddy(i + 1) * h(i) / 6
                         - _ddy(i) * h(i) / 3;
            _data[i].d = y(i);
        }
    }
    }
    _valid = true;
}
