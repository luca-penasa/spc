#ifndef SPC_NN_INTERPOLATOR_H
#define SPC_NN_INTERPOLATOR_H

#include <vector>
#include <limits>
#include <stdlib.h>
#include <iostream>

namespace spc
{

/** \brief A simple Nearest Neighbors interpolator.
 * \note Given two vectors x and y plus a vector of of x positions at which to
 *evaluate the interpolator the class
 * performs nninterpolation. Class makes use of linear search for finding
 *neighbors
 *
 * \author Luca Penasa
 */
class InterpolatorNN

{

public:
    /** \brief Constructor.
     */
    InterpolatorNN();

    /** \brief Set input x vector.
            * \param[in] x x vector
     */
    void setX(const std::vector<float> &x)
    {
        x_ = x;
        n_in_ = x_.size();
        check_input_size();
    }

    /** \brief Set y input vector
     *  \param[in] y input vector
     */
    void setY(const std::vector<float> &y)
    {
        y_ = y;
        check_input_size();
    }

    /** \brief Set both x and y input vectors.
     * \param[in] x input vector
     * \param[in] y input vector
     */
    void setXY(const std::vector<float> &x, const std::vector<float> &y)
    {
        setX(x);
        setY(y);
        check_input_size();
    }

    /** \brief Set the x vector of positions at which evaluate the interpolator
     *  \param[in] new_x vector of x poisitions at which to evaluate the
     * interpolator
     */
    void setNewX(const std::vector<float> &new_x)
    {
        new_x_ = new_x;
        n_out_ = new_x_.size();
    }

    /** \brief Get NN interpolated y array as computed with compute method.
     */
    std::vector<float> getNewY()
    {
        if (is_computed_ == true) {
            return new_y_;
        } else {
            std::cout
                << "You need to compute output first! Use compute() method."
                << std::endl;
            return new_y_;
        }
    }

    /** \brief Compute the output y vector.
     */
    int compute();

private:
    // input data
    std::vector<float> x_;
    std::vector<float> y_;

    std::vector<float> new_x_;
    std::vector<float> new_y_;

    int n_in_;
    int n_out_;

    bool is_input_ok_;

    bool is_computed_;

    /** \brief Check if input is ok for computations
     * \note size of x and y must be the same and != zero. Also a new_x array
     * must be present and not null.
     */
    bool check_input_size()

    {
        if ((x_.size() == y_.size()) && (x_.size() != 0)
            && (new_x_.size() != 0)) {
            is_input_ok_ = true;
            return true;
        } else
            return false;
    }

    /** \brief Get the nearest point id for a given x value
     * \param[in] value the x value for which to search for
     */
    int getNearestID(const float &value);

    /** \brief Get the nearest y value for a given x value
     * \param[in] value the y value for which to search for
    */
    float getNearestValue(const float &value);

    /** \brief Reset bool indicator that are checked before computations
     */
    void reset()
    {
        is_input_ok_ = false;
        is_computed_ = false;
    }
};

} // close nspace

#endif
