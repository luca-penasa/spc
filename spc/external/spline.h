/*  dynamo:- Event driven molecular dynamics simulator
    http://www.dynamomd.org
    Copyright (C) 2011  Marcus N Campbell Bannerman <m.bannerman@gmail.com>

    This program is free software: you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    version 3 as published by the Free Software Foundation.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// From
// https://github.com/toastedcrumpets/DynamO/blob/master/src/magnet/magnet/math/spline.hpp

#pragma once

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <exception>

#include <spc/core/macros.h>

namespace ublas = boost::numeric::ublas;
namespace magnet
{
namespace math
{
class SPC_LIB_API Spline : private std::vector<std::pair<double, double>>
{
public:
    // The boundary conditions available
    enum BC_type {
        FIXED_1ST_DERIV_BC,
        FIXED_2ND_DERIV_BC,
        PARABOLIC_RUNOUT_BC
    };

    enum Spline_type {
        LINEAR,
        CUBIC
    };

    // Constructor takes the boundary conditions as arguments, this
    // sets the first derivative (gradient) at the lower and upper
    // end points
    Spline()
        : _valid(false), _BCLow(FIXED_2ND_DERIV_BC),
          _BCHigh(FIXED_2ND_DERIV_BC), _BCLowVal(0), _BCHighVal(0), _type(CUBIC)
    {
    }

    typedef std::vector<std::pair<double, double>> base;
    typedef base::const_iterator const_iterator;

    // Standard STL read-only container stuff
    const_iterator begin() const
    {
        return base::begin();
    }
    const_iterator end() const
    {
        return base::end();
    }
    void clear()
    {
        _valid = false;
        base::clear();
        _data.clear();
    }
    size_t size() const
    {
        return base::size();
    }
    size_t max_size() const
    {
        return base::max_size();
    }
    size_t capacity() const
    {
        return base::capacity();
    }
    bool empty() const
    {
        return base::empty();
    }

    // Add a point to the spline, and invalidate it so its
    // recalculated on the next access
    inline void addPoint(double x, double y)
    {
        _valid = false;
        base::push_back(std::pair<double, double>(x, y));
    }

    // Reset the boundary conditions
    inline void setLowBC(BC_type BC, double val = 0)
    {
        _BCLow = BC;
        _BCLowVal = val;
        _valid = false;
    }

    inline void setHighBC(BC_type BC, double val = 0)
    {
        _BCHigh = BC;
        _BCHighVal = val;
        _valid = false;
    }

    void setType(Spline_type type)
    {
        _type = type;
        _valid = false;
    }

    // Check if the spline has been calculated, then generate the
    // spline interpolated value
    double operator()(double xval);

private:
    ///////PRIVATE DATA MEMBERS
    struct SplineData
    {
        double x, a, b, c, d;
    };
    // vector of calculated spline data
    std::vector<SplineData> _data;
    // Second derivative at each point
    ublas::vector<double> _ddy;
    // Tracks whether the spline parameters have been calculated for
    // the current set of points
    bool _valid;
    // The boundary conditions
    BC_type _BCLow, _BCHigh;
    // The values of the boundary conditions
    double _BCLowVal, _BCHighVal;

    Spline_type _type;

    ///////PRIVATE FUNCTIONS
    // Function to calculate the value of a given spline at a point xval
    inline double splineCalc(std::vector<SplineData>::const_iterator i,
                             double xval)
    {
        const double lx = xval - i->x;
        return ((i->a * lx + i->b) * lx + i->c) * lx + i->d;
    }

    inline double lowCalc(double xval)
    {
        const double lx = xval - x(0);

        if (_type == LINEAR)
            return lx * _BCHighVal + y(0);

        const double firstDeriv = (y(1) - y(0)) / h(0)
                                  - 2 * h(0) * (_data[0].b + 2 * _data[1].b)
                                    / 6;

        switch (_BCLow) {
        case FIXED_1ST_DERIV_BC:
            return lx * _BCLowVal + y(0);
        case FIXED_2ND_DERIV_BC:
            return lx * lx * _BCLowVal + firstDeriv * lx + y(0);
        case PARABOLIC_RUNOUT_BC:
            return lx * lx * _ddy[0] + lx * firstDeriv + y(0);
        }
        throw std::runtime_error("Unknown BC");
    }

    inline double highCalc(double xval)
    {
        const double lx = xval - x(size() - 1);

        if (_type == LINEAR)
            return lx * _BCHighVal + y(size() - 1);

        const double firstDeriv
            = 2 * h(size() - 2) * (_ddy[size() - 2] + 2 * _ddy[size() - 1]) / 6
              + (y(size() - 1) - y(size() - 2)) / h(size() - 2);

        switch (_BCHigh) {
        case FIXED_1ST_DERIV_BC:
            return lx * _BCHighVal + y(size() - 1);
        case FIXED_2ND_DERIV_BC:
            return lx * lx * _BCHighVal + firstDeriv * lx + y(size() - 1);
        case PARABOLIC_RUNOUT_BC:
            return lx * lx * _ddy[size() - 1] + lx * firstDeriv + y(size() - 1);
        }
        throw std::runtime_error("Unknown BC");
    }

    // These just provide access to the point data in a clean way
    inline double x(size_t i) const
    {
        return operator[](i).first;
    }
    inline double y(size_t i) const
    {
        return operator[](i).second;
    }
    inline double h(size_t i) const
    {
        return x(i + 1) - x(i);
    }

    // Invert a arbitrary matrix using the boost ublas library
    template <class T>
    bool InvertMatrix(ublas::matrix<T> A, ublas::matrix<T> &inverse)
    {
        using namespace ublas;

        // create a permutation matrix for the LU-factorization
        permutation_matrix<std::size_t> pm(A.size1());

        // perform LU-factorization
        int res = lu_factorize(A, pm);
        if (res != 0)
            return false;

        // create identity matrix of "inverse"
        inverse.assign(ublas::identity_matrix<T>(A.size1()));

        // backsubstitute to get the inverse
        lu_substitute(A, pm, inverse);

        return true;
    }

    // This function will recalculate the spline parameters and store
    // them in _data, ready for spline interpolation
    void generate();
};
}
}
