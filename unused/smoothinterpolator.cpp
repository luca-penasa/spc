#include "smoothinterpolator.h"

template<typename T>
LinearInterpolator<T>::LinearInterpolator()
{
}

template<typename T>
int
LinearInterpolator<T>::updateInternalTables()
{
    //check if the x_ vector is made of equally-spaced segments
    segments_.clear();
    segments_.resize(n_points_ - 1); //we have n - 1 segments between points.
    rel_distances_.resize(n_points_ - 1); //we have n - 1 segments between points.

    //for each point for which to estimate the smooth function
    for (int i = 0 ; i < new_x_.size() ; ++i)
    {
        T position = new_x_[i]; //its position
        if (position <= x_start_)
        {
            //the point is lower than the possible range
            lower_points_.push_back(i);
        }
        else if (position > x_end_ )
        {
            //the point is over the upper bound for valid interpolation
            upper_points_.push_back(i);
        }
        else
        {
            //now establish to which segment is pertinent.
            T decpart, intpart;


            decpart = modf( (position - x_start_) / step_ , &intpart);
            int segment_id = floor(intpart);


            segments_.at(segment_id).push_back(i);
            rel_distances_.at(segment_id).push_back(decpart);
            //get its distance

        }
    }

}


//relative positioin from 0 to 1 within a given segment. 1 and 0 gives the same value than using the adjacent segments.
template<typename T>
T
LinearInterpolator<T>::evaluateInterpolator(int &segment, T &relative_position)
{
       T w = 1/relative_position;// (1 - cos(relative_position * M_PI)) * 0.5;
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
LinearInterpolator<T>::compute()
{
    for (int i = 0 ; i < segments_.size(); ++i)
    {
        vector<T> values = evaluateInterpolatorForSegment(i);
        tmp_out_.push_back(values);
    }

    //write the effective output
    for (int i = 0; i < segments_.size(); ++i)
    {
        for (int j= 0; j<segments_.at(i).size(); ++j)
        {
            int original_id = segments_.at(i).at(j);
            new_y_.at(original_id) = tmp_out_.at(i).at(j);
        }
    }
}

template class LinearInterpolator<float>;
template class LinearInterpolator<double>;

