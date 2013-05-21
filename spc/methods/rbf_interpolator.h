#ifndef SPC_RBF_INTERPOLATOR_H
#define SPC_RBF_INTERPOLATOR_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>
namespace spc
{
template<typename nType>
class RBFInterpolator
{
	
	typedef Eigen::Matrix<nType, Eigen::Dynamic, Eigen::Dynamic> MatrixT;
	typedef Eigen::Matrix<nType, Eigen::Dynamic, 1> VectorT;
	
	
	
public:

	
	
	RBFInterpolator(){} ;
	
	nType
	evaluateRbf(VectorT &point);
	
	
	
	void 
	updateAll()
	{
		updateMatrixOfDistances();
		updateMatrixOfCoefficients();
		computeNodes();
	};
	
	void
	setInputPoints(MatrixT &points)
	{
		
		points_ = points;
		
		updateInputPointsInfo();
	};
	
	
	//Used for 1-d case
	void 
	setInputPoints(const std::vector<nType> &points)
	{
		//we need to map the std::vector to an Eigen matrix type
		
		points_ = Eigen::Matrix<nType, Eigen::Dynamic, 1>::Map(&points[0], points.size());
		updateInputPointsInfo();
	}
	
	void 
	updateInputPointsInfo()
	{
		n_points_ = points_.rows();
		n_dim_ = points_.cols();
		d_matrix_.resize(n_points_, n_points_);
		c_matrix_.resize(n_points_, n_points_);
		w_vector_.resize(n_points_);
	}
	

	
	void
	setInputValues(VectorT &values)
	{
		values_ = values;
	};
	
	//So we can set input with a std::vector
	void
	setInputValues(std::vector<nType> &values)
	{
		values_ = Eigen::Matrix<nType, Eigen::Dynamic, 1>::Map(&values[0], values.size());
		
	};
	
	
	
	
// private:
	MatrixT points_;
	VectorT values_;
	int n_points_;
	int n_dim_;
	
	//Matrix of distances
	MatrixT d_matrix_;
	
	//Matrix of coefficients
	MatrixT c_matrix_;
	
	//Vector of nodes weights
	VectorT w_vector_;
	
	
	
	inline nType thinPlate(nType &r)
	{
		if (r == 0)
			return 0.0;
		else
			return pow(r,2) * log(r);
	};
	
	void
	updateMatrixOfDistances();
	
	void 
	updateMatrixOfCoefficients();
		
	void 
	computeNodes();

	
};
//#include <spc/methods/rbf_interpolator.hpp>
} //end namespace
#endif
