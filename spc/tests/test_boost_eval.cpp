//--------------------------------------------------------------------------
//
// Copyright (C) 2011, 2012, 2013 Rhys Ulerich
// Copyright (C) 2012, 2013 The PECOS Development Team
// Please see http://pecos.ices.utexas.edu for more information on PECOS.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
//--------------------------------------------------------------------------

#include <cmath>
#include <limits>
#include <iterator>

#include <boost/spirit/version.hpp>
#if !defined(SPIRIT_VERSION) || SPIRIT_VERSION < 0x2010
#error "At least Spirit version 2.1 required"
#endif
#include <boost/math/constants/constants.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

namespace expression
{

namespace
{ // anonymous

struct lazy_pow_
{
    template <typename X, typename Y> struct result
    {
        typedef X type;
    };

    template <typename X, typename Y> X operator()(X x, Y y) const
    {
        return std::pow(x, y);
    }
};

struct lazy_ufunc_
{
    template <typename F, typename A1> struct result
    {
        typedef A1 type;
    };

    template <typename F, typename A1> A1 operator()(F f, A1 a1) const
    {
        return f(a1);
    }
};

struct lazy_bfunc_
{
    template <typename F, typename A1, typename A2> struct result
    {
        typedef A1 type;
    };

    template <typename F, typename A1, typename A2>
    A1 operator()(F f, A1 a1, A2 a2) const
    {
        return f(a1, a2);
    }
};

template <class T> T max_by_value(const T a, const T b)
{
    return std::max(a, b);
}

template <class T> T min_by_value(const T a, const T b)
{
    return std::min(a, b);
}

} // end namespace anonymous

template <typename FPT, typename Iterator>
struct grammar : boost::spirit::qi::grammar
                 <Iterator, FPT(), boost::spirit::ascii::space_type>
{

    // symbol table for constants like "pi"
    struct constant_
        : boost::spirit::qi::symbols
          <typename std::iterator_traits<Iterator>::value_type, FPT>
    {
        constant_()
        {
            this->add("digits", std::numeric_limits<FPT>::digits)(
                "digits10", std::numeric_limits
                <FPT>::digits10)("e", boost::math::constants::e<FPT>())(
                "epsilon", std::numeric_limits
                <FPT>::epsilon())("pi", boost::math::constants::pi<FPT>());
        }
    } constant;

    // symbol table for unary functions like "abs"
    struct ufunc_
        : boost::spirit::qi::symbols
          <typename std::iterator_traits<Iterator>::value_type, FPT (*)(FPT)>
    {
        ufunc_()
        {
            this->add("abs", static_cast<FPT (*)(FPT)>(&std::abs))(
                "acos", static_cast<FPT (*)(FPT)>(&std::acos))(
                "asin", static_cast<FPT (*)(FPT)>(&std::asin))(
                "atan", static_cast<FPT (*)(FPT)>(&std::atan))(
                "ceil", static_cast<FPT (*)(FPT)>(&std::ceil))(
                "cos", static_cast<FPT (*)(FPT)>(&std::cos))(
                "cosh", static_cast<FPT (*)(FPT)>(&std::cosh))(
                "exp", static_cast<FPT (*)(FPT)>(&std::exp))(
                "floor", static_cast<FPT (*)(FPT)>(&std::floor))(
                "log10", static_cast<FPT (*)(FPT)>(&std::log10))(
                "log", static_cast<FPT (*)(FPT)>(&std::log))(
                "sin", static_cast<FPT (*)(FPT)>(&std::sin))(
                "sinh", static_cast<FPT (*)(FPT)>(&std::sinh))(
                "sqrt", static_cast<FPT (*)(FPT)>(&std::sqrt))(
                "tan", static_cast<FPT (*)(FPT)>(&std::tan))(
                "tanh", static_cast<FPT (*)(FPT)>(&std::tanh));
        }
    } ufunc;

    // symbol table for binary functions like "pow"
    struct bfunc_ : boost::spirit::qi::symbols
                    <typename std::iterator_traits<Iterator>::value_type,
                     FPT (*)(FPT, FPT)>
    {
        bfunc_()
        {
            this->add("atan2", static_cast<FPT (*)(FPT, FPT)>(&std::atan2))(
                "max", static_cast<FPT (*)(FPT, FPT)>(&max_by_value))(
                "min", static_cast<FPT (*)(FPT, FPT)>(&min_by_value))(
                "pow", static_cast<FPT (*)(FPT, FPT)>(&std::pow));
        }
    } bfunc;

    boost::spirit::qi::rule
        <Iterator, FPT(), boost::spirit::ascii::space_type> expression,
        term, factor, primary;

    grammar() : grammar::base_type(expression)
    {
        using boost::spirit::qi::real_parser;
        using boost::spirit::qi::real_policies;
        real_parser<FPT, real_policies<FPT>> real;

        using boost::spirit::qi::_1;
        using boost::spirit::qi::_2;
        using boost::spirit::qi::_3;
        using boost::spirit::qi::no_case;
        using boost::spirit::qi::_val;

        boost::phoenix::function<lazy_pow_> lazy_pow;
        boost::phoenix::function<lazy_ufunc_> lazy_ufunc;
        boost::phoenix::function<lazy_bfunc_> lazy_bfunc;

        expression = term[_val = _1] >>
                     *(('+' >> term[_val += _1]) | ('-' >> term[_val -= _1]));

        term = factor[_val = _1]
               >> *(('*' >> factor[_val *= _1]) | ('/' >> factor[_val /= _1]));

        factor = primary[_val = _1]
                 >> *(("**" >> factor[_val = lazy_pow(_val, _1)]));

        primary = real[_val = _1] | '(' >> expression[_val = _1] >> ')'
                  | ('-' >> primary[_val = -_1]) | ('+' >> primary[_val = _1])
                  | (no_case[ufunc] >> '(' >> expression
                     >> ')')[_val = lazy_ufunc(_1, _2)]
                  | (no_case[bfunc] >> '(' >> expression >> ',' >> expression
                     >> ')')[_val = lazy_bfunc(_1, _2, _3)]
                  | no_case[constant][_val = _1];
    }
};

template <typename FPT, typename Iterator>
bool parse(Iterator &iter, Iterator end, const grammar<FPT, Iterator> &g,
           FPT &result)
{
    return boost::spirit::qi::phrase_parse(iter, end, g,
                                           boost::spirit::ascii::space, result);
}

} // end namespace expression

/////////////
// Test cases
/////////////

#include <string>
#include <boost/math/special_functions/fpclassify.hpp>

#define BOOST_TEST_MODULE expression
#include <boost/test/included/unit_test.hpp>

const expression::grammar<double, std::string::const_iterator> eg;
const double close_enough = std::numeric_limits<double>::epsilon();

#define EXPRTEST(casename, expr, expected)                                     \
    BOOST_AUTO_TEST_CASE(casename)                                             \
    {                                                                          \
        const std::string s = expr;                                            \
        std::string::const_iterator iter = s.begin();                          \
        std::string::const_iterator end = s.end();                             \
        double result;                                                         \
        BOOST_REQUIRE(parse(iter, end, eg, result));                           \
        BOOST_CHECK(iter == end);                                              \
        BOOST_CHECK_CLOSE(result, (expected), close_enough);                   \
    }

EXPRTEST(literal1, "1.234", 1.234)
EXPRTEST(literal2, "4.2e2", 420)
EXPRTEST(literal3, "5e-01", 0.5)
EXPRTEST(literal4, "-3", -3)
EXPRTEST(literal5, "pi", boost::math::constants::pi<double>())
EXPRTEST(literal6, "epsilon", std::numeric_limits<double>::epsilon())
EXPRTEST(literal7, "digits", std::numeric_limits<double>::digits)
EXPRTEST(literal8, "digits10", std::numeric_limits<double>::digits10)
EXPRTEST(literal9, "e", boost::math::constants::e<double>())

EXPRTEST(basicop1, " 2 +\t3\n", 5) // Whitespace ignored
EXPRTEST(basicop2, " 2 -\t3\n", -1)
EXPRTEST(basicop3, " 2 *\t3\n", 6)
EXPRTEST(basicop4, " 2 /\t3\n", 2. / 3.) // Double division
EXPRTEST(basicop5, " 2 ** 3\n", 8)

EXPRTEST(pemdas1, "2*3+4*5", 2 * 3 + 4 * 5)
EXPRTEST(pemdas2, "2*(3+4)*5", 2 * (3 + 4) * 5)
EXPRTEST(pemdas3, "2**3+4", std::pow(2, 3) + 4)
EXPRTEST(pemdas4, "2**2**-3", std::pow(2., std::pow(2., -3.)))

EXPRTEST(unary1, "-(2)", -2)
EXPRTEST(unary2, "-(-2)", 2)
EXPRTEST(unary3, "+(-2)", -2)
EXPRTEST(unary4, "+(+2)", 2)

EXPRTEST(func_abs, "abs (-1.0)", std::abs(-1.0))
EXPRTEST(func_acos, "acos( 1.0)", std::acos(1.0))
EXPRTEST(func_asin, "asin( 1.0)", std::asin(1.0))
EXPRTEST(func_atan, "atan( 1.0)", std::atan(1.0))
EXPRTEST(func_ceil, "ceil( 0.5)", std::ceil(0.5))
EXPRTEST(func_cos, "cos ( 1.0)", std::cos(1.0))
EXPRTEST(func_cosh, "cosh( 1.0)", std::cosh(1.0))
EXPRTEST(func_exp, "exp ( 1.0)", std::exp(1.0))
EXPRTEST(func_floor, "floor(0.5)", std::floor(0.5))
EXPRTEST(func_log, "log ( 1.0)", std::log(1.0))
EXPRTEST(func_log10, "log10(0.5)", std::log10(0.5))
EXPRTEST(func_sin, "sin ( 1.0)", std::sin(1.0))
EXPRTEST(func_sinh, "sinh( 1.0)", std::sinh(1.0))
EXPRTEST(func_sqrt, "sqrt( 1.0)", std::sqrt(1.0))
EXPRTEST(func_tan, "tan ( 1.0)", std::tan(1.0))
EXPRTEST(func_tanh, "tanh( 1.0)", std::tanh(1.0))

EXPRTEST(func_pow, "pow (2.0, 3.0)", std::pow(2.0, 3.0))
EXPRTEST(func_max, "max (2.0, 3.0)", std::max(2.0, 3.0))
EXPRTEST(func_min, "min (2.0, 3.0)", std::min(2.0, 3.0))
EXPRTEST(func_atan2, "atan2(2.0, 3.0)", std::atan2(2.0, 3.0))

BOOST_AUTO_TEST_CASE(parse_inf)
{
    const std::string s = "inf";
    std::string::const_iterator iter = s.begin();
    std::string::const_iterator end = s.end();
    double result;
    BOOST_REQUIRE(parse(iter, end, eg, result));
    BOOST_CHECK(iter == end);
    BOOST_CHECK((boost::math::isinf)(result));
}

BOOST_AUTO_TEST_CASE(parse_infinity)
{
    const std::string s = "infinity";
    std::string::const_iterator iter = s.begin();
    std::string::const_iterator end = s.end();
    double result;
    BOOST_REQUIRE(parse(iter, end, eg, result));
    BOOST_CHECK(iter == end);
    BOOST_CHECK((boost::math::isinf)(result));
}

BOOST_AUTO_TEST_CASE(parse_nan)
{
    const std::string s = "nan";
    std::string::const_iterator iter = s.begin();
    std::string::const_iterator end = s.end();
    double result;
    BOOST_REQUIRE(parse(iter, end, eg, result));
    BOOST_CHECK(iter == end);
    BOOST_CHECK((boost::math::isnan)(result));
}
