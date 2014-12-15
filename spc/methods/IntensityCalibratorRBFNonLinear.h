#ifndef INTENSITYCALIBRATORRBFNONLINEAR_H
#define INTENSITYCALIBRATORRBFNONLINEAR_H


#include <spc/elements/RBFModel.h>
#include <spc/elements/calibration/DataHolder.h>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>




namespace spc
{
namespace calibration
{

//! a material is just a collector of keypoints
//! taken from just a single material
class Material: public DataHolder
{
public:
	spcTypedefSharedPtrs(Material)

	Material(const float factor, int orig_material_id)
	{
		factor_ = factor;
		original_material_id_ = orig_material_id;
	}

	Material(const float factor, int orig_material_id,  const DataHolder &other) : DataHolder(other)
	{
		factor_ = factor;
		original_material_id_ = orig_material_id;
	}

	bool isLocked() const
	{
		return is_locked_;
	}

	void lock()
	{
		is_locked_ = true;
	}

	bool unlock()
	{
		is_locked_ = false;
	}

	spcGetMacro(Factor, factor_, float)

	void setFactor(const float & factor)
	{
		if (!isLocked())
			factor_ = factor;
	}


protected:
	float factor_ = spcNANMacro;
	bool is_locked_ = false;
	int original_material_id_ = std::numeric_limits<int>::quiet_NaN();
};

template <typename _Scalar, int NX = Dynamic, int NY = Dynamic> struct Functor
{
	typedef _Scalar Scalar;
	enum {
		InputsAtCompileTime = NX,
		ValuesAtCompileTime = NY
	};
	typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
	typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
	typedef Eigen::Matrix
	<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

	const int m_inputs, m_values;

	Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime)
	{
	}
	Functor(int inputs, int values) : m_inputs(inputs), m_values(values)
	{
	}

	int inputs() const
	{
		return m_inputs;
	}
	int values() const
	{
		return m_values;
	}


};



class IntensityCalibratorRBFNonLinear
{
public:
	IntensityCalibratorRBFNonLinear();


	spcSetObjectMacro(Model, model_, RBFModel<float>)
	spcGetObjectMacro(Model, model_, RBFModel<float>)

	void setCalibrationData(DataHolder::Ptr data)
	{

		all_keypoints_ = data;

		//! we split the data in "materials"
		Eigen::VectorXi materials = data->getUniqueMaterials();

		for (int i = 0; i < materials.rows(); i++)
		{
			int mat_id = materials(i);

			if (mat_id == -1) // -1 is unclassified stuff, each material will be made of just 1 keypoint
			{
				DataHolder::Ptr unclassified = data->getKeypointsOnMaterial(-1);
				for (KeyPoint::Ptr kpoint: unclassified->getData())
				{

					Material::Ptr newmat (new Material(0,  mat_id));
					newmat->appendKeypoint(kpoint);
					materials_.push_back(newmat);

//					newmat->lock(); // keep locked for DEBUG
				}

			}

			else // all other materials are treated the same actually
			{
				DataHolder::Ptr keys = data->getKeypointsOnMaterial(mat_id);
				Material::Ptr newmat (new Material(0,  mat_id, *keys));

				if (mat_id == fixed_material_id_)
				{
					newmat->lock();
					newmat->setFactor(value_for_fixed_material_);
				}

				materials_.push_back(newmat);
			}
		}

	}

	void initMaterialsFactorFromCurrentModel()
	{
		for (Material::Ptr mat: materials_)
		{
			if (mat->isLocked())
				continue;

			Eigen::VectorXf all_factors (mat->getTotalNumberOfObservations());
			int counter = 0;
			for (Observation::Ptr ob: mat->getAllObservations())
			{
				float prediction = model_->operator ()(ob->getAsEigenPoint());
				if (!std::isfinite(prediction) | prediction == 0)
				{
					prediction = 1;
				}
				all_factors(counter ++) = ob->intensity / prediction;
			}

			float avg = all_factors.sum() / all_factors.rows();

			mat->setFactor(avg);

			LOG(INFO) << "material factor set to " <<avg;

		}
	}



	Eigen::VectorXf getActiveMaterialFactors() const
	{
		Eigen::VectorXf out;
		for (Material::Ptr mat: materials_)
		{
			if (!mat->isLocked()) // locked materials will not be optimized
				out.push_back(mat->getFactor());
		}
		return out;
	}


	void setMaterialFactors(const Eigen::VectorXf factors)
	{
		int counter = 0;
		for (Material::Ptr mat: materials_)
		{
			if(!mat->isLocked()) // only if the material is NON locked
				mat->setFactor(factors(counter++));


		}
	}


	spcSetMacro(FixedMaterialId, fixed_material_id_, size_t)
	spcGetMacro(FixedMaterialId, fixed_material_id_, size_t)

	spcSetMacro(ValueOfFixedMat, value_for_fixed_material_, float)
	spcGetMacro(ValueOfFixedMat, value_for_fixed_material_, float)




	void optimize()
	{
		disableMaterialsWithLessThanObservations(3);
		initMaterialsFactorFromCurrentModel();
		int outsize = all_keypoints_->getTotalNumberOfObservations();

		Eigen::VectorXf factors = getActiveMaterialFactors();

		LOG(INFO) << "found " <<factors.rows() << " active materials";


		my_functor Functor(this, model_->getCoefficients().rows() + factors.size(), outsize);

		Eigen::NumericalDiff<my_functor> numDiff(Functor);
		LevenbergMarquardt<Eigen::NumericalDiff<my_functor>, float> lm(numDiff);

		lm.parameters.maxfev = 400000;

		LOG(INFO) << "lev marq initialized";

		Eigen::VectorXf coeffs = model_->getCoefficients();


		LOG(INFO) << "init coeffs " << coeffs.transpose();
		LOG(INFO) << "init factors " << factors.transpose();

		coeffs.conservativeResize(coeffs.rows() + factors.rows());
		coeffs.tail(factors.rows()) = factors;

		//		return;

		LOG(INFO) << "starting minimization";
		int info = lm.minimize(coeffs);

		LOG(INFO) << "Done. Exit status: " << info;
	}

protected:

	struct my_functor : Functor<float>
	{
		my_functor(IntensityCalibratorRBFNonLinear * calibrator, int xsize, int ysize) : Functor<float>(xsize, ysize)
		{
			calibrator_ = calibrator;
		}

		int operator()(const VectorXf &x, VectorXf &fvec) const
		{
			//			LOG(INFO)<< "CALLED functor, new pars: " << x.transpose();

			int n_rbf_coeffs = calibrator_->getModel()->getCoefficients().rows();
			Eigen::VectorXf rbf_coeffs = x.head(n_rbf_coeffs);
			Eigen::VectorXf mat_factors = x.tail(x.rows() - n_rbf_coeffs);


			LOG(INFO) << "rbf-coeffs " << rbf_coeffs.rows();

			calibrator_->getModel()->setCoefficients(rbf_coeffs);

			calibrator_->setMaterialFactors(mat_factors);

			fvec = calibrator_->getSquaredResiduals();

//						LOG(INFO) << "current pars: " << x.transpose();

			LOG(INFO)<< "CALLED functor, residuals: " << sqrt(fvec.sum() / fvec.size());

			return 0;
		}

		/// a pointer to the estimator itself
		IntensityCalibratorRBFNonLinear * calibrator_;
	};

	void disableMaterialsWithLessThanObservations(int n_obs)
	{
		for (Material::Ptr mat: materials_)
		{
			if (mat->getTotalNumberOfObservations() < n_obs)
				mat->lock();
		}
	}




	Eigen::VectorXf getSquaredResiduals() const
	{
		Eigen::VectorXf out;



		for (Material::Ptr material: materials_)
		{

			if (material->isLocked())
			{
				for (Observation::Ptr ob: material->getAllObservations())
				{
					out.push_back(0);
				}
			}
			else
			{
				for (Observation::Ptr ob: material->getAllObservations())
				{

					float predicted = model_->operator ()(ob->getAsEigenPoint()) * material->getFactor();

					if (!std::isfinite(predicted))
					{
						LOG(WARNING) << "predicted not finite!";
					}

					float diff  = predicted - ob->intensity; // predicted - observed

					out.push_back(diff * diff);
				}
			}
		}

		return out;
	}


	RBFModel<float>::Ptr model_;

	//! each dataholder in the vector will be a single
	//! material
	std::vector<Material::Ptr> materials_;


	//! here the keypoints are stored un-ordered
	DataHolder::Ptr all_keypoints_;


	size_t fixed_material_id_ = 0;

	Eigen::VectorXf materials_factors_;


	float value_for_fixed_material_ = 1;
};


}
} // end nspaces


#endif // INTENSITYCALIBRATORRBFNONLINEAR_H
