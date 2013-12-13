
#include <spc/common/common.h>
#include <spc/methods/linear_interpolator.h>

namespace spc
{

template<typename T>
LinearInterpolator<T>::LinearInterpolator(): degree_(2)
{
}

template<typename T>
int
LinearInterpolator<T>::updateInternalTables()
{

    segments_.clear();
    segments_.resize(n_points_-1); //we have n - 1 segments between points.
    rel_distances_.resize(n_points_-1); //we have n - 1 segments between points.

    //for each point for which to estimate the smooth function
    for (int i = 0 ; i < new_x_.size() ; ++i)
    {
        T position = new_x_[i]; //its position
        if (position < x_start_)
        {
            //the point is lower than the possible range
            //            std::cout << "Point " << i << "lower of defined range.\n";
            lower_points_.push_back(i);

        }
        else if (position == x_start_) // we put it in the first segment
        {
            segments_.at(0).push_back(i);
            rel_distances_.at(0).push_back(0.0f);
            std::cout << "Point " << i << "equal to lower bound.\n";
        }
        else if (position > x_end_ )
        {
            //the point is over the upper bound for valid interpolation
            //            std::cout << "Point " << i << "upper of defined range.\n";
            upper_points_.push_back(i);
        }
        else if (position == x_end_)
        {
            // is clearly pertinent to the last segment,
            // if we would not do this the next else would put this point in a new segment.
            segments_.at(segments_.size()-1).push_back( i );
            rel_distances_.at(segments_.size()-1).push_back(1.0f); //dist is clearly 1 - end of the segment
            //            std::cout << "Point " << i << "equal to upper bound.\n";
        }
        else
        {
            //now establish to which segment is pertinent.
            T decpart, intpart;


            decpart = modf( (position - x_start_) / step_ , &intpart);
            int segment_id = floor(intpart);

            //            std::cout << "segment id " << segment_id << std::endl;
            segments_.at(segment_id).push_back(i);
            rel_distances_.at(segment_id).push_back(decpart);


        }
    }


}


//relative positioin from 0 to 1 within a given segment. 1 and 0 gives the same value than using the adjacent segments.
template<typename T>
T
LinearInterpolator<T>::evaluateInterpolator(int &segment, T &relative_position)
{
    T w = relative_position;// (1 - cos(relative_position * M_PI)) * 0.5;
    return( y_.at(segment)  * ( 1 - w )+ y_.at(segment + 1)  * w );

}

template <typename T>
vector<T>
LinearInterpolator<T>::evaluateInterpolatorForSegment(int &segment_id)
{
    vector<int> pertinent_ids = segments_.at(segment_id);
    vector<T> relative_distances = rel_distances_.at(segment_id);
    vector<T> values;
    values.resize(pertinent_ids.size());

    //for each pertinent id
    for (int i = 0; i < pertinent_ids.size(); ++i)
    {
        int id = pertinent_ids.at(i);
        //its relative distance
        T dist = relative_distances.at(i);
        //std::cout << dist << std::endl;

        T value = evaluateInterpolator(segment_id, dist);
        values[i] = value;

    }
    return values;
}


template<typename T>
void
LinearInterpolator<T>::evaluateInterpolatorLowerPoints()
{
    int n = degree_ + 1; //number of points we will need to estimate a polynomials for estimating the out-of bound function

    std::vector<T> small_x(n), small_y(n);

    int nn = 0;
    for (auto &i: small_x)
        i = x_start_ + step_ * nn++;

    nn = 0;
    for (auto &i: small_y)
        i = y_.at(nn++);

    auto tmp_x_e = spc::stdVectorToEigen(small_x);
    auto tmp_y_e = spc::stdVectorToEigen(small_y);

    auto poly = spc::polyfit(tmp_x_e, tmp_y_e, n-1);

    //using this polynomial we evaluate all the points out of lower bound
    std::vector <T> x_to_evaluate(lower_points_.size()); //container
    std::transform(lower_points_.begin(), lower_points_.end(), x_to_evaluate.begin(), [&](int id){return this->new_x_[id];});

    auto x_to_evaluate_e = spc::stdVectorToEigen(x_to_evaluate);
    auto values = spc::polyval(poly, x_to_evaluate_e);
    std::cout << poly << std::endl;


    //now put thes values back to their right place in new_y_
    int count = 0;
    for (auto id: lower_points_)
        new_y_.at(id) = values.coeff(count++);

}

template<typename T>
void
LinearInterpolator<T>::evaluateInterpolatorUpperPoints()
{
    int n = degree_ + 1; //number of points we will need to estimate a polynomials for estimating the out-of bound function

    std::vector<T> small_x(n), small_y(n);

    int nn = 0;
    for (auto &i: small_x)
        i = x_start_ + step_*(n_points_ - 1) - step_ * nn++;

    nn = n_points_ -1;
    for (auto &i: small_y)
        i = y_.at(nn--);

    auto tmp_x_e = spc::stdVectorToEigen(small_x);
    auto tmp_y_e = spc::stdVectorToEigen(small_y);

    auto poly = spc::polyfit(tmp_x_e, tmp_y_e, n-1);

    //using this polynomial we evaluate all the points out of lower bound
    std::vector <T> x_to_evaluate(upper_points_.size()); //container
    std::transform(upper_points_.begin(), upper_points_.end(), x_to_evaluate.begin(), [&](int id){return this->new_x_[id];});

    auto x_to_evaluate_e = spc::stdVectorToEigen(x_to_evaluate);
    auto values = spc::polyval(poly, x_to_evaluate_e);
    std::cout << poly << std::endl;


    //now put thes values back to their right place in new_y_
    int count = 0;
    for (auto id: upper_points_)
        new_y_.at(id) = values.coeff(count++);

}



template<typename T>
void
LinearInterpolator<T>::compute()
{
    updateInternalTables();



    for (int i = 0 ; i < segments_.size(); ++i)
    {
        vector<T> values = evaluateInterpolatorForSegment(i);
        tmp_out_.push_back(values);
    }



    //write the effective output
    for (int i = 0; i < segments_.size(); ++i)
    {
        for (int j= 0; j< segments_.at(i).size(); ++j)
        {
            int original_id = segments_.at(i).at(j);
            new_y_.at(original_id) = tmp_out_.at(i).at(j);
        }
    }

    //evaluate interpolator for upper and lower
    evaluateInterpolatorUpperLowerPoints();


}

template class LinearInterpolator<float>;
template class LinearInterpolator<double>;

}
