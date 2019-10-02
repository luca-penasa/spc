#ifndef SMOOTHINTERPOLATOR_H
#define SMOOTHINTERPOLATOR_H

#include <vector>
//#include <Common/lidarlib.h>

using namespace std;


template <typename T>
class LinearInterpolator
{
public:
    //constructor
    LinearInterpolator();

    //set x and y data
    void setXY(vector<T> &y, T & step, T & min_x)
    {
        y_ = y;

        //additional infos
        x_start_ = min_x;
        step_ = step;
        n_points_ = y.size();
        x_end_ = x_start_ + (n_points_ - 1) * step_;

    }

    void setNewX(vector<T> &new_x)
    {
        assert( new_x.size()  != 0); //need to be not zero!

        new_x_ = new_x;
        new_y_.resize(new_x_.size());
    }

    vector<T> evaluateInterpolatorForSegment(int &segment_id);

    void compute();

    vector<T> getNewY()
    {
        return this->new_y_;
    }

    int updateInternalTables();

private:


    T evaluateInterpolator(int &segment, T &relative_position);

    //data to be interpolated
    vector<T> y_;

    //results of interpolation
    vector<T> new_x_;
    vector<T> new_y_;

    //additional infos on the data
    T step_; //in the case of equally spaces
    T x_start_; //first point in x_ ( min(x_) )
    T x_end_;
    int n_points_;

    //each id of the points for which we want to evaluate the interpolator
    //are stored here, assigning each id to one of the segments defined in x_
    vector< vector<int> > segments_;
    vector< vector<T> > rel_distances_;
    vector< vector<T> > tmp_out_;


    //the points out-of-range
    vector<int> lower_points_;
    vector<int> upper_points_;
};

#endif // SMOOTHINTERPOLATOR_H
