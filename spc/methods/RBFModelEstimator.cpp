#include <spc/methods/RBFModelEstimator.h>
#include <spc/methods/RBFModelEstimator.hpp>

namespace spc
{


template <typename T>
void RBFModelEstimator<T>::appendEqualityConstrainForPoints(const RBFModelEstimator::PointsT &points, const RBFModelEstimator::VectorT &values, const RBFModelEstimator::VectorT &weights)
{
	for (size_t i  = 0; i < points.rows() - 1; ++i)
	{
		PointT p1 =points.row(i);
		PointT p2 =points.row(i+1);

		T v1 = values(i);
		T v2 = values(i+1);


		//! we will use the minimum weight between the two

		T w;
		int status = -1;
		if (weights.size() != 0) // this should mean some weights were actually passed
		{
			w = std::min( weights(i), weights(i+1) );
			status = this->appendEqualityConstrain(p1, v1, p2, v2, w);
		}
		else
			status = this->appendEqualityConstrain(p1, v1, p2, v2);

		if (status != 1)
		{
			LOG(ERROR) << "cannot add constrains. see log. Nothing done";
			return;
		}


	}
}
template <typename T>
int RBFModelEstimator<T>::appendEqualityConstrain(const RBFModelEstimator<T>::PointT &point1, const T &value1, const RBFModelEstimator::PointT &point2, const T &value2,
												  const T &weight)
{

	if (A_.rows() == 0)
	{
		LOG(ERROR) << "looks like you are adding the constrains befoe the A matrix. "
					  "Please add your eq. constrains only after the system has been setup. "
					  "Nothin done.";
		return -1;
	}




	VectorT a = model_->getPredictorVector(point1) / value1;
	VectorT b = model_->getPredictorVector(point2) / value2;

	VectorT Aline = a .array() - b.array();

	if (weight != 1)
		Aline = Aline * weight;

	if (A_.cols() != Aline.rows())
	{
		LOG(ERROR) << "you are adding the constrain on a matrix with a different number of columns. "
					  "this may means that you are using the classical rbf but constrains are not allowed"
					  " with that method. Nothing done";

		return - 1;
	}

	//            LOG(INFO) << "new line: " << Aline.transpose();

	//            LOG(A_>bottomRows(1).rows())
	A_.conservativeResize(A_.rows() + 1, Eigen::NoChange );
	A_.bottomRows(1) = Aline.transpose();

	b_.push_back(0);

	return 1;

}

template <typename T>
void RBFModelEstimator<T>::autosetNodes(const Eigen::VectorXi &n_nodes, RBFModelEstimator<T>::PointsT input_points)
{

	if (input_points.size() == 0)
		input_points = points_;

	CHECK (input_points.size() != 0) << "set the points before calling this method";
	CHECK (n_nodes.rows() == input_points.cols()) << "dimensions and number of splits does not match";

	DLOG(INFO) << "autosetting nodes";

	size_t total_nodes_ = n_nodes.array().prod();
	DLOG(INFO) << "total number of nodes " << total_nodes_ << std::endl;

	std::vector<VectorT> nod_coords;
	for (int i =0; i < input_points.cols(); ++i)
	{
		T min = input_points.col(i).minCoeff();
		T max = input_points.col(i).maxCoeff();


		VectorT coords = VectorT::LinSpaced(n_nodes(i), min, max);
		nod_coords.push_back(coords);
	}

	MatrixT  nodes = meshgrid<T>(nod_coords);
	model_->setNodes( nodes );

	DLOG(INFO) << "nodes automatically set";


	//        autosetScales();

}


template <typename T>
void RBFModelEstimator<T>::autosetScales(const size_t fixed_dimension, RBFModelEstimator<T>::PointsT input_points)
{

	if (input_points.size() == 0)
		input_points = points_;


	CHECK(input_points.size() != 0) <<"set the input points before to call this method";
	CHECK(fixed_dimension <= input_points.cols()) << "you have to set a fixed dimension index lower or equal than the dimensions";

	DLOG(INFO) << "autodefining scales" ;

	VectorT m, std;
	input_points.meanAndStd(m, std);
	DLOG(INFO) << "mean: \n" << m ;
	DLOG(INFO) << "std: \n" << std ;

	VectorT scales(input_points.cols());

	T fixed_std = std(fixed_dimension);

	for (int i = 0; i <input_points.cols(); ++i)
	{
		if (i == fixed_dimension)
			scales(i) = 1;
		else
		{
			T ratio = fixed_std / std(i);
			scales(i) = ratio;
		}
	}

	model_->setScales(scales);

	DLOG(INFO) << "scales defined";
}

template <typename T>
void RBFModelEstimator<T>::autosetSigma()
{
	CHECK(model_->getNodes().size() != 0) << "nodes must be set before calling this function";
	VectorT nn_distances(model_->getNodes().rows()); // nearest neighbor distance for each node

	for (int i = 0 ; i < model_->getNodes().rows(); ++i)
	{
		PointT n = model_->getNodes().row(i);
		VectorT distances = model_->getSqDistanceFromNodes(n);


		distances = distances.nonZeroCoeffs();
		T min_dist = distances.minCoeff();
		nn_distances(i) = sqrt(min_dist);
	}

	T avg =  nn_distances.mean();
	LOG(INFO) << "Sigma autoset to " << avg;

	this->model_->setSigma(avg);




	DLOG(INFO) << "nearest neighbor distances " << nn_distances.T();
}

template<typename T>
int RBFModelEstimator<T>::initProblem()
{
	if (!doChecksBeforeInit())
	{
		LOG(ERROR) << "some error occurred. cannot init the problem. please see log info.";
		return -1;
	}

	//////////// checks are ok... go ahead!
	DLOG(INFO) << "Checks passed!";

	size_t n_nodes = model_->getNumberOfNodes();
	size_t n_polys = model_->getNumberOfpolynomialTerms();

	// number of columns is the same with both methods
	// notice that in the classical rbf nodes_ corresponds to points_
	size_t n_cols = n_nodes + n_polys;

	// number of rows changes depending on the method
	// in the classical implementation a square matrix will be formed
	// while in the method with user-selected nodes the matrix will be rectangular
	// with size n_points +
	// and solution is provided by least squares
	size_t n_rows;

	if (classical_rbf_)
		n_rows = getNumberOfPoints() + n_polys;
	else // non classical
	{
		// then we have two cases
		if (lambda_ == 0 ) // no regularization
			n_rows = getNumberOfPoints();
		else // with regularization
			n_rows = getNumberOfPoints() + n_nodes;
	}

	// resize A
	try {
		A_.resize(n_rows, n_cols);
	}
	catch (const std::bad_alloc&)
	{
		LOG(WARNING) << "out of memory allocating A";
		return -1;
	}





	// b_ MUST have same to A_.rows();
	b_.resize(n_rows);

	// init them with zeros.
	A_.fill(0);
	b_.fill(0);


	DLOG(INFO) << "Going to populate A";


	// fill b vector with the values. the remaining part will be zeros...
	b_.head(values_.rows()) = values_;

	if (hasWeights())
	{
		b_.head(values_.rows()).array() *= weights_.array();
		DLOG(INFO) << "Wightin done";
	}
	else
	{
		DLOG(INFO) << "NOT weighted";
	}





	// now fill A with the predictors
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < getNumberOfPoints(); ++i) // a row for each point
	{
		VectorT pre  = model_->getPredictorVector(points_.row(i));
		A_.row(i) = pre;


		if (hasWeights())
		{
			A_.row(i).array() *= weights_(i);
		}
	}

	DLOG(INFO) << "A populated";


	if (classical_rbf_)
	{
		DLOG(INFO) << "copy transposed matrix";

		// copy the transposed part of polynomials at the bottom
		// we use an intermediate matrix for clarity
		MatrixT tmp = A_.topRightCorner(getNumberOfPoints(), n_polys);
		A_.bottomLeftCorner(n_polys, getNumberOfPoints()) = tmp.transpose();
	}




	// now add regularization
	if (lambda_ != 0)
	{
		DLOG(INFO) << "Add regularization";

		if (classical_rbf_) // add an identinty * lambda_ to the upper left corner
		{
			A_.topLeftCorner(getNumberOfPoints(), getNumberOfPoints()) += MatrixT::Identity(getNumberOfPoints(), getNumberOfPoints()) * lambda_;
		}
		else
		{
			MatrixT tikhonov = MatrixT::Identity(n_nodes, n_nodes) * sqrt(lambda_);
			A_.bottomLeftCorner(n_nodes, n_nodes) = tikhonov;
		}
	}

	DLOG(INFO) << "Init done";


	return 1;

}
template <typename T>
int RBFModelEstimator<T>::solveProblem()
{
	if (A_.cols() > A_.rows())
	{
		LOG(WARNING)     << "looks like your problem is underdetermined. Consider reducing the number of polynomials terms \n"
							"or the number of nodes you are using, or add more data points for the fitting \n"
							"you can also overcome this problem using regularization (if you are not using it yet) but it is not suggested.";

		return -1;
	}

	if(A_.size() == 0)
	{
		LOG(WARNING)<< "A matrix is empty";
		return -1;
	}

	if(b_.size() == 0)
	{
		LOG(WARNING)<< "b vector is empty";
		return -1;
	}

	DLOG(INFO) << "going to solve the system";

	VectorT coeffs;
	// do the linear system solving
	if (A_.rows() == A_.cols()) {
		// solve via pivoting
		coeffs = A_.colPivHouseholderQr().solve(b_);
		model_->setCoefficients(coeffs);
		DLOG(INFO) << "system correctly solved via Housolder QR";


	}
	else if (A_.rows() > A_.cols()) {
		// solving via SVD
		Eigen::JacobiSVD< MatrixT> svd = A_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

		//             LOG(INFO) << "mat U" << svd.matrixU() ;
		//             LOG(INFO) << "mat V" << svd.matrixV() ;

		//             A_(0,0) = std::numeric_limits<float>::quiet_NaN();
		LOG(INFO) << "A finiteness " << A_.finiteness().count() << " over " << A_.size();
		LOG(INFO) << "b finiteness " << b_.finiteness().count() << " over " << b_.size();



		LOG(INFO) << "singular values: " << svd.singularValues();
		LOG(INFO) << "condition number: " << svd.singularValues()(0) / svd.singularValues()(A_.cols() -1);

		LOG(WARNING) << "NONZERO SV: " << svd.nonzeroSingularValues() << " over " << A_.cols();
		coeffs =svd.solve(b_);
		LOG(INFO) << "solution is " << coeffs.transpose();
		model_->setCoefficients(coeffs);
		LOG(INFO) << "system correctly solved via JacobiSVD";
	}
	else
	{
		LOG(WARNING) << "Cannot solve the system cause constraints are missing in the A matrix (rows)";
		return -1; // unsuccessfull;
	}

	return 1;
}

//template class InterpolatorRBF<float, 1>;
//template class InterpolatorRBF<float, 2>;



//template class InterpolatorRBF<double, 1>;
//template class InterpolatorRBF<double, 2>;
//template class InterpolatorRBF<double, 3>;


template class RBFModelEstimator<float>;

}
