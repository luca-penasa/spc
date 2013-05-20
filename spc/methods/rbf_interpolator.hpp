#ifndef RBFINTERPOLATOR_IMPL_H_
#define RBFINTERPOLATOR_IMPL_H_

#include <spc/methods/rbf_interpolator.h>




template<typename nType> nType
RBFInterpolator<nType>::evaluateRbf(VectorT &point)
{
	nType distance;
	nType sumw = 0.0;
	nType sum = 0.0;
	for (int i = 0; i < n_points_; i++)
	{
		VectorT p = points_.row(i);
		distance = (p-point).norm();
		nType value = thinPlate(distance);
		sumw += w_vector_(i) * value;
		sum += value;
	}
	return sumw ;
}


template<typename nType> void
RBFInterpolator<nType>::updateMatrixOfDistances()
{
	for (int i = 0; i < n_points_; ++i)
	{
		VectorT p1 = points_.row(i);
		
		for (int j = 0; j <= i; ++j)
		{
			if (i == j)
			{
				d_matrix_(i,j) = 0;
			}
			VectorT p2 = points_.row(j);
			d_matrix_(i,j) = d_matrix_(j,i) =  (p1 - p2).norm();
		}
	}
}

template<typename nType> void
RBFInterpolator<nType>::updateMatrixOfCoefficients()
{
	for (int i = 0; i < n_points_; ++i)
	{
		for (int j = 0; j < i; ++j)
		{
			c_matrix_(i,j) = c_matrix_(j,i) = thinPlate(d_matrix_(j,i));
		}
	}
}

template<typename nType> void
RBFInterpolator<nType>::computeNodes()
{
	
	//Solve the linear system
	w_vector_ = c_matrix_.colPivHouseholderQr().solve(values_);
	
}



#endif


