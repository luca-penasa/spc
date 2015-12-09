#ifndef SELECTIONEXTRACTOR_H
#define SELECTIONEXTRACTOR_H

#include <spc/elements/SelectionBase.h>
#include <spc/elements/templated/PointSetBase.h>

namespace spc
{


template<typename ElementT, typename IdT = size_t>
class SelectionExtractor
{
public:

    spcTypedefSharedPtrs(SelectionExtractor)

    SelectionExtractor()
    {

    }

    void setSelection(const typename SelectionBase<ElementT>::ConstPtr selection)
    {
        selection_ = selection;
    }

    void setInputSet(const typename PointSetBase<typename ElementT::Scalar>::ConstPtr data)
    {
        in_set_ = data;
    }

    int compute ()
    {

        if (in_set_ == NULL)
        {
            LOG(ERROR) << "you must provide an input set on which to check the selection";
            return -1;
        }

        if (selection_ == NULL)
        {
            LOG(ERROR) << "you must provide a selection to use for extracting the points";
            return -1;
        }

//#ifdef USE_OPENMP
//#pragma omp parallel for private(selection_)
//#endif
        // not parallelized for now cause we does no know beforehand the number of inside points
        for (size_t i= 0; i <(size_t) in_set_->getNumberOfPoints(); ++i)
        {
            bool inside = selection_->contains(in_set_->getPoint(i));

            if (inside)
                inside_points_.push_back(i);
            else
                outside_points_.push_back(i);

        }

        percentage_inside_ = (float) inside_points_.size() / (float) in_set_->getNumberOfPoints() * 100.0f;

        return 1;
    }

    std::vector<IdT> getInsideIds() const
    {
        return inside_points_;
    }

    std::vector<IdT> getOutsideIds() const
    {
        return outside_points_;
    }

    float getPercentageInside() const
    {
        return percentage_inside_;
    }




protected:
    typename SelectionBase<ElementT>::ConstPtr selection_;
   typename  PointSetBase<typename ElementT::Scalar>::ConstPtr in_set_;

    std::vector<IdT> inside_points_;
    std::vector<IdT> outside_points_;

    float percentage_inside_ = std::numeric_limits<float>::quiet_NaN();




};



}//end nspace

#endif // SELECTIONEXTRACTOR_H
