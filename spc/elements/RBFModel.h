#ifndef RBFMODEL_H
#define RBFMODEL_H

#include <spc/core/spc_eigen.h>
#include <spc/elements/RBFKernelFactory.h>
#include <spc/core/polynomials.hpp>
namespace spc
{


/** RBF models typically uses gaussian, multiquadric, inverse multiquadric
 *  polyharmonic splines and thin plate splines
 *  Some of them can be obtained from RBFKernelFactory
 */
template <typename T>
class RBFModel: public ElementBase
{
public:

//    friend class InterpolatorRbf;

    SPC_ELEMENT(RBFModel<T>)

    typedef Eigen::Matrix<T, -1, -1> PointsT;
    typedef Eigen::Matrix<T, -1, 1> VectorT;
    typedef Eigen::Matrix<T, -1, 1> PointT;
    typedef Eigen::Matrix<T, -1, -1> MatrixT;




	RBFModel(): kernel_(new GaussianApproxRBF<T>(1.0))
    {

    }

    size_t getNumberOfpolynomialTerms() const
    {
        return n_polys_;
    }


    size_t getNumberOfNodes() const
    {
        return nodes_.rows();
    }


    void setNodes(const PointsT &nodes)
    {
        DLOG(INFO) << "setting nodes";
        if (hasScales())
        {
            // scale the nodes

            PointsT scaled_nodes = nodes.cwiseProduct(scales_.transpose().replicate(nodes.rows(), 1));
            nodes_ = scaled_nodes;
        }
        else
        {
            nodes_ = nodes;
        }

        dim_ = nodes_.cols();

        updateNPolyTerms();
        DLOG(INFO) << "nodes are ok";

    }

    void updateNPolyTerms()
    {
        n_polys_ = pow(poly_order_+1, dim_); // precomputed number of poly-terms
    }


    PointsT getNodes() const
    {
        return nodes_;
    }

    void setSigma(const T& sigma)
    {
        DLOG(INFO) << "set sigma order to " << sigma;

        kernel_->setScale(sigma);
    }

    T getSigma () const
    {
        return kernel_->getSigma();
    }


    void setKernel(typename RBFKernelFactory<T>::RBF_FUNCTION kernel, const T scale = T(1))
    {
            kernel_ =RBFKernelFactory<T>::create(kernel, scale);
    }

    void setPolyOrder(const size_t & order)
    {
        DLOG(INFO) << "set poly order to " << order;
        poly_order_ = order;
        updateNPolyTerms();
    }

    VectorT getSqDistanceFromNodes(const PointT &point) const
    {
        PointsT diff;
        // point to nodes difference
        diff = nodes_.rowwise() - point.transpose();

        // squared norm of that difference.
        // i.e. the squared distance from nodes or points
        // for the input point
        VectorT sq_dist =  diff.rowwise().squaredNorm();

        return sq_dist;
    }

    VectorT getRbfPredictorPart(const PointT &point) const
    {
        VectorT w = getSqDistanceFromNodes(point);
        for (int i = 0; i < w.rows(); ++i)
            w(i) = kernel_->eval(w(i));

        return w;
    }

    VectorT getPolyPredictorPart(const PointT &point) const
    {
        return getPolynomialVariables<T>(point, poly_order_);
    }


    VectorT getPredictorVector(const PointT &point) const
    {


        PointT p;
        if (hasScales())
            p = applyScales(point);
        else
            p = point;


        VectorT rbf =getRbfPredictorPart(p);
        VectorT pol =getPolyPredictorPart(p);
        VectorT out(getNumberOfNodes() + getNumberOfpolynomialTerms());




        out << rbf, pol;

        if (out.finiteness().count() != out.size())
        {
            LOG(FATAL) << "error of finiteness: " << out.transpose();
        }

        return out;

    }
    PointT applyScales(const PointT & p) const
    {
        return scales_.array() * p.array();
    }

    bool hasScales() const
    {
        if (scales_.size() != 0)
            return true;
        else
            return false;
    }

    void setScales(const VectorT &scales)
    {
        scales_ = scales;
    }

    VectorT getScales() const
    {
        return scales_;
    }

	T op2(const PointT &p) const
	{
		return operator ()(p);
	}


    T operator()(const PointT &p) const
    {
        VectorT v = getPredictorVector(p);
        return v.dot(coeffs_);
    }



    void operator() (const PointsT &points, VectorT &values)
    {
        values.resize(points.rows());
#ifdef USE_OPENMP
        #pragma omp parallel for
#endif
        for (int i = 0; i < points.rows(); ++i)
        {
            values(i) = operator ()(points.row(i));
        }
    }

    void splittedEvaluator(const PointsT & points, MatrixT & values)
    {

        DLOG(INFO) << "calling splitted evaluator";
        values.resize(points.rows(), 2);
#ifdef USE_OPENMP
        #pragma omp parallel for
#endif
        for (int i = 0; i < points.rows(); ++i)
        {
            PointT p;
            if (hasScales())
            {
                p = applyScales(points.row(i));
            }
            else
            {
                p = points.row(i);
            }

            values(i, 0) = getPolyCoefficients().dot(getPolyPredictorPart(p));
            values(i, 1) = getRbfCoefficients().dot(getRbfPredictorPart(p));
        }

        DLOG(INFO) << "Done!";

    }


	void resetCoefficients(const T & value)
	{
		coeffs_.fill(value);
	}

    void setCoefficients(const VectorT & coeffs)
    {
//		CHECK(coeffs.rows() == coeffs_.rows()) << "Wrong coefficients dimension";
        coeffs_ = coeffs;
    }

    VectorT getCoefficients() const
    {
        return coeffs_;
    }

    VectorT getPolyCoefficients() const
    {
        return coeffs_.tail(getNumberOfpolynomialTerms());
    }

    VectorT getRbfCoefficients() const
    {
        return coeffs_.head(getNumberOfNodes());
    }


    bool isValid() const
    {
        if (coeffs_.rows() == nodes_.rows())
            return true;
        else
            return false;
    }

	size_t getDimensionality() const
    {
        return dim_;
    }

protected:
    //! the positions of the nodes
    //! if no nodes are provided a classical RBF
    //! with one node for each point will be used
    PointsT nodes_;

    //! Vector of nodes coefficients - almost compeltely defines the RBF
    VectorT coeffs_;

    //! the kernel
    typename RBFBase<T>::Ptr kernel_;

    //! user options
    //! set the polynomial order using setPolyOrder()
    //! n_polys_ will be automatically updated
    size_t poly_order_ = 0;
    size_t n_polys_ = 1;

    //! the dimensions of points space
    size_t dim_ = 0;

    //! scaling factors for each variable
    VectorT scales_;


private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::ElementBase>(this),
           CEREAL_NVP(nodes_),
           CEREAL_NVP(coeffs_),
           CEREAL_NVP(kernel_),
           CEREAL_NVP(poly_order_),
           CEREAL_NVP(n_polys_),
           CEREAL_NVP(dim_),
           CEREAL_NVP(scales_));
    }

};

} // end nspace

#endif // RBFMODEL_H
