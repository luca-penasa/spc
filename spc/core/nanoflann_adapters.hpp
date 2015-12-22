#pragma once
#ifndef NANOFLANN_ADAPTERS_HPP
#define NANOFLANN_ADAPTERS_HPP

#include <nanoflann.hpp>
#include <spc/core/spc_eigen.h>

using namespace Eigen;
using namespace spc;


template <class Derived, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = size_t>
struct NanoFlannEigenMatrixAdaptor
{
    typedef NanoFlannEigenMatrixAdaptor<Derived,DIM,Distance,IndexType> self_t;
    typedef typename Derived::Scalar              ScalarT;
    typedef typename Distance::template traits<ScalarT,self_t>::distance_t metric_t;
    typedef nanoflann::KDTreeSingleIndexAdaptor< metric_t,self_t,DIM,IndexType>  index_t;

    spcTypedefSharedPtrs(self_t)

    index_t* index; //! The kd-tree index for the user to call its methods as usual with any other FLANN index.

    /// Constructor: takes a const ref to the matrix object with the data points
    NanoFlannEigenMatrixAdaptor(const Eigen::Ref<const Derived> &mat, const int leaf_max_size = 20) : m_data_matrix(mat)
    {
        const size_t dims = mat.cols();

        DLOG(INFO) << "nanoflann eigen matrix adaptor creation with dimension: " << dims;
        if (DIM>0 && static_cast<int>(dims)!=DIM)
            throw std::runtime_error("Data set dimensionality does not match the 'DIM' template argument");
        index = new index_t( dims, *this /* adaptor */, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size ) );
        index->buildIndex();
        DLOG(INFO) << "nanoflann index done.";
    }


private:
    /** Hidden copy constructor, to disallow copying this class (Not implemented) */
    NanoFlannEigenMatrixAdaptor(const self_t&);
public:

    ~NanoFlannEigenMatrixAdaptor() {
        delete index;
    }

    const Eigen::Ref<const Derived>  &m_data_matrix;

    int radiusSearch(const Eigen::Ref<const Eigen::Matrix<ScalarT, -1, 1>> &position,
                     const ScalarT &sq_radius,
                     std::vector<std::pair<IndexType, ScalarT>> &matches,
                     const nanoflann::SearchParams &pars = nanoflann::SearchParams()) const
    {

        DLOG(INFO) << "doing search at " << position.transpose();
        return index->radiusSearch(position.data(), sq_radius, matches, pars);
    }

    /** @name Interface expected by KDTreeSingleIndexAdaptor
    * @{ */

    const self_t & derived() const {
        return *this;
    }
    self_t & derived()       {
        return *this;
    }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const {
        return m_data_matrix.rows();
    }

    // Returns the L2 distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline ScalarT kdtree_distance(const ScalarT *p1, const size_t idx_p2,size_t size) const
    {
        ScalarT s=0;
        for (size_t i=0; i<size; i++) {
            const ScalarT d= p1[i]-m_data_matrix.coeff(idx_p2,i);
            s+=d*d;
        }
        return s;
    }

    // Returns the dim'th component of the idx'th point in the class:
    inline ScalarT kdtree_get_pt(const size_t idx, int dim) const {
        return m_data_matrix(idx,dim);
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX &bb) const {
        return false;
    }



}; // end of KDTreeEigenMatrixAdaptor






template <class Derived, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = size_t>
struct NanoFlannEigenBlockAdaptor
{
    typedef NanoFlannEigenBlockAdaptor<Derived,DIM,Distance,IndexType> self_t;
    typedef typename Derived::Scalar              ScalarT;
    typedef typename Distance::template traits<ScalarT,self_t>::distance_t metric_t;
    typedef nanoflann::KDTreeSingleIndexAdaptor< metric_t,self_t,DIM,IndexType>  index_t;

    spcTypedefSharedPtrs(self_t)

    index_t* index; //! The kd-tree index for the user to call its methods as usual with any other FLANN index.

    /// Constructor: takes a const ref to the matrix object with the data points
    NanoFlannEigenBlockAdaptor(const Eigen::Block<Derived> block, const int leaf_max_size = 20) : m_data_block(block)
    {
        const size_t dims = block.cols();

        DLOG(INFO) << "nanoflann eigen block adaptor creation with dimension: " << dims;
        index = new index_t( dims, *this /* adaptor */, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size) );
        index->buildIndex();
        DLOG(INFO) << "nanoflann index done.";
    }


private:
    /** Hidden copy constructor, to disallow copying this class (Not implemented) */
    NanoFlannEigenBlockAdaptor(const self_t&);
public:

    ~NanoFlannEigenBlockAdaptor() {
        delete index;
    }

    const Eigen::Block<Derived>  m_data_block;

    /**
     * @brief radiusSearch perfom search around radius
     * @param position
     * @param sq_radius
     * @param indices
     * @param distances_sq
     * @param pars
     * @return
     */
    int radiusSearch(const Eigen::Ref<const Eigen::Matrix<ScalarT, -1, 1>> &position,
                     const ScalarT &sq_radius,
                     std::vector<IndexType> &indices,
                     std::vector<ScalarT> &distances_sq,
                     const nanoflann::SearchParams &pars = nanoflann::SearchParams()) const
    {

        std::vector<std::pair<IndexType,ScalarT> > matches;

        int n = index->radiusSearch(position.data(), sq_radius, matches, pars);

        indices.resize(n);
        distances_sq.resize(n);

        size_t id = 0;
        for (std::pair<IndexType, ScalarT> pair: matches)
        {
            indices.at(id) = pair.first;
            distances_sq.at(id) = pair.second;

            id++;
        }

        return n;
    }


    /**
     * @brief knnSearch do a nighbor search based on the number of nearest neighbors
     * @param position
     * @param num_closest
     * @param indices
     * @param distances_sq
     * @return  true if something found, false if not
     */
    bool knnSearch (const Eigen::Ref<const Eigen::Matrix<ScalarT, -1, 1>> &position,
                    const size_t num_closest,
                    std::vector<IndexType> &indices,
                    std::vector<ScalarT> &distances_sq,
                    const nanoflann::SearchParams &pars = nanoflann::SearchParams()) const
    {
        indices.resize(num_closest);
        distances_sq.resize(num_closest);
        nanoflann::KNNResultSet<ScalarT> resultSet(num_closest);
        resultSet.init(&indices[0], &distances_sq[0] );
        return index->findNeighbors(resultSet, position.data(), pars);
    }



    /** @name Interface expected by KDTreeSingleIndexAdaptor
    * @{ */

    const self_t & derived() const {
        return *this;
    }
    self_t & derived()       {
        return *this;
    }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const {
        return m_data_block.rows();
    }

    // Returns the L2 distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline ScalarT kdtree_distance(const ScalarT *p1, const size_t idx_p2,size_t size) const
    {
        ScalarT s=0;
        for (size_t i=0; i<size; i++) {
            const ScalarT d= p1[i]-m_data_block(idx_p2,i);
            s+=d*d;
        }
        return s;
    }

    // Returns the dim'th component of the idx'th point in the class:
    inline ScalarT kdtree_get_pt(const size_t idx, int dim) const {
        return m_data_block(idx,dim);
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX &bb) const {
        return false;
    }



}; // end of KDTreeEigenMatrixAdaptor

#endif // NANOFLANN_ADAPTERS_HPP
