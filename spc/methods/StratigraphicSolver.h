#pragma once
#ifndef STRATIGRAPHICSOLVER_H
#define STRATIGRAPHICSOLVER_H

//#include <spc/elements/StratigraphicPositionableElement.h>
#include <spc/elements/StratigraphicModelBase.h>
#include <spc/elements/StratigraphicConstrain.h>
#include <spc/elements/StratigraphicPositionableElement.h>
#include <spc/core/spc_eigen.h>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
namespace spc {

template <typename _Scalar, int NX = Dynamic, int NY = Dynamic>
struct FunctorSolver {
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    FunctorSolver()
        : m_inputs(InputsAtCompileTime)
        , m_values(ValuesAtCompileTime)
    {
    }
    FunctorSolver(int inputs, int values)
        : m_inputs(inputs)
        , m_values(values)
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

class StratigraphicSolver {
public:
    spcTypedefSharedPtrs(StratigraphicSolver)

    enum MODE { NON_LINEAR = 0,
                LINEAR,
                MIXED};

protected:
    struct my_functor : FunctorSolver<float> {
        my_functor(StratigraphicSolver* solver, int xsize, int ysize)
            : FunctorSolver<float>(xsize, ysize)
        {
            solver_ = solver;
        }

        int operator()(const VectorXf& x, VectorXf& fvec) const
        {
            LOG(INFO) << "CALLED functor, new pars: " << x.transpose();

            int counter = 0;
            for (spc::StratigraphicModelBase::Ptr mod : solver_->models_)


            {
                if (!mod->getIsFreezed())
                {


                mod->setStratigraphicShift(x(counter++));


                    if ((mod->getIsElastic()) & (solver_->solve_elastic_))
                        mod->setElasticParameter(x(counter++));

                }
            }

            fvec = solver_->getSquaredResiduals();

            LOG(INFO) << "CALLED functor, residuals: " << fvec.transpose();

            return 0;
        }

        /// a pointer to the estimator itself
        StratigraphicSolver* solver_;
    };

public:
    StratigraphicSolver()
    {
    }

    /**
     * @brief reset restores the StratigraphicSolver as new
     */
	void reset()
	{
		models_.clear();
		constrains_.clear();
		positionables_.clear();
	}

	Eigen::VectorXf getSquaredResiduals() const
	{
		Eigen::VectorXf out;
		//        out.resize(constrains_.size() + positionables_.size());

		//        size_t i = 0;
		for (spc::StratigraphicConstrain::Ptr c : constrains_) {
			Eigen::VectorXf res = c->getSquaredResiduals();

			LOG(INFO) << "for constrain " << c->getElementName() << " " << res.transpose();
			out.conservativeResize(out.rows() + res.rows());
			out.tail(res.rows()) = res;

			LOG(INFO) << "out is: " << out.transpose();
		}

		for (StratigraphicPositionableElement::Ptr p : positionables_) {
			out.conservativeResize(out.rows() + 1);
			float res = p->getSquaredResidual();
			out.tail(1)(0) = res;
			LOG(INFO) << "residual for " << p->getElementName() << res;
		}

		LOG(INFO) << "current residuals: \n" << out;

		return out;
	}

	void solve()
	{
		if (mode_ == NON_LINEAR) {
			LOG(INFO) << "computing non-linear solution";
			solve_nonlinear();
		}
		else if (mode_ == LINEAR) {
			LOG(INFO) << "computing linear solution";
			solve_linear();
		}

		else if (mode_ == MIXED) {
			LOG(INFO) << "computing linear and non-linear mixed solution";

			solve_mixed();
		}
	}

    spcSetGetMacro(Mode, mode_, MODE);

	void setInputFromChildren(ElementBase::Ptr root_element)
	{

		LOG(INFO) << "going to extract inputs from childs";
		models_.clear();
		constrains_.clear();
		positionables_.clear();

		models_ = root_element->findElementsThatAre<spc::StratigraphicModelBase>(&spc::StratigraphicModelBase::Type);

		for (StratigraphicModelBase::Ptr mod : models_) {
			std::vector<spc::ElementBase::Ptr> pos = mod->findElementsThatAre(&spc::StratigraphicPositionableElement::Type);

			for (spc::ElementBase::Ptr el : pos) {
				spc::StratigraphicPositionableElement::Ptr p = spcDynamicPointerCast<spc::StratigraphicPositionableElement>(el);
				if (p->getManual()) //! < only elements set to manual can provide a constrain
				{
					positionables_.push_back(p);
					LOG(INFO) << "found a positionable: " << el->getElementName();
				}
			}

			std::vector<spc::ElementBase::Ptr> constrains = mod->findElementsThatAre(&spc::StratigraphicConstrain::Type);

			for (ElementBase::Ptr el : constrains) {
				spc::StratigraphicConstrain::Ptr p = spcDynamicPointerCast<StratigraphicConstrain>(el);
				constrains_.push_back(p);

				LOG(INFO) << "found a constrain: " << el->getElementName();
			}
		}

		LOG(INFO) << "extraction done ";
	}

protected:
	void solve_nonlinear()
	{
		VectorXf init_x;

		int counter = 0;
		for (spc::StratigraphicModelBase::Ptr mod : models_) {
			if (!mod->getIsFreezed()) // do not optimize anythin on freezed models
			{
				init_x.conservativeResize(init_x.rows() + 1);
				init_x(counter++) = mod->getStratigraphicShift();
				if (mod->getIsElastic() & solve_elastic_) {
					init_x.conservativeResize(init_x.rows() + 1);
					init_x(counter++) = mod->getElasticParameter();
				}
			}
		}
		LOG(INFO) << "init x: " << init_x.transpose();

		int residual_size = this->getSquaredResiduals().rows();

		//        int outsize = constrains_.size() + positionables_.size();

		my_functor Functor(this, init_x.rows(), residual_size);

		Eigen::NumericalDiff<my_functor> numDiff(Functor);
		LevenbergMarquardt<Eigen::NumericalDiff<my_functor>, float> lm(numDiff);
		//    lm.parameters.factor = 1;

		LOG(INFO) << "lev marq initialized";

		LOG(INFO) << "starting minimization";
		int info = lm.minimize(init_x);

		LOG(INFO) << "Done. Exit status: " << info;
	}

	void solve_linear()
	{
		Eigen::MatrixXf A;
		Eigen::VectorXf b;

		// \todo we are assuming the constrained models are in models_ find a better way!
		size_t n_models = 0;

		for (auto mod : models_) // count unfreezed models
		{
			if (!mod->getIsFreezed())
				n_models += 1;
		}

		size_t n_constr = constrains_.size();
		size_t n_pos = positionables_.size();

		std::map<spc::StratigraphicModelBase::Ptr, size_t> model_to_par_id_map;
		std::map<size_t, spc::StratigraphicModelBase::Ptr> par_id_to_model_map;

		size_t count = 0;
		for (auto mod : models_) {
			if (!mod->getIsFreezed()) {
				model_to_par_id_map[mod] = count; // only unfreezed models will be considered
				par_id_to_model_map[count] = mod;
				count++;
			}
		}

		A.resize(0, n_models);
		//    b.resize();

		A.fill(0.0);

		size_t row_counter = 0;

		for (spc::StratigraphicPositionableElement::Ptr pos : positionables_) {
			spc::StratigraphicModelBase::Ptr mymodel = pos->getStratigraphicModel();

			if (!mymodel->getIsFreezed()) // do nothing if the model is freezed
			{
				float ps = pos->predictStratigraphicPositionFromModel();

				float shift = mymodel->getStratigraphicShift();

				float d = ps - shift;

				size_t id = model_to_par_id_map.at(mymodel);

				A.conservativeResize(A.rows() + 1, Eigen::NoChange);
				A.row(row_counter).fill(0);

				b.conservativeResize(b.rows() + 1);

				A(row_counter, id) = 1;
				b(row_counter) = -d;

				row_counter++;
			}
		}

		for (spc::StratigraphicConstrain::Ptr cons : constrains_) {
			spc::StratigraphicPositionableElement::Ptr pos1 = cons->getVertices().at(0);
			spc::StratigraphicPositionableElement::Ptr pos2 = cons->getVertices().at(1);

			float sp1 = pos1->getStratigraphicPosition();
			float sp2 = pos2->getStratigraphicPosition();

			spc::StratigraphicModelBase::Ptr model1 = pos1->getStratigraphicModel();
			spc::StratigraphicModelBase::Ptr model2 = pos2->getStratigraphicModel();

			if (model1->getIsFreezed() && model2->getIsFreezed()) // do nothing
				continue;

			else if (model1->getIsFreezed()) // model 2 is the unfreezed one and model 1 is instead freezed
			{
				size_t id = model_to_par_id_map.at(model2); // id of the unfreezen

				A.conservativeResize(A.rows() + 1, Eigen::NoChange);
				A.row(row_counter ).fill(0);
				A(row_counter, id) = 1;

				float shift1 = model1->getStratigraphicShift(); // of unfreezed
				float shift2 = model2->getStratigraphicShift(); // of freezed

				float d1 = sp1 - shift1; // of unfreezed
				float d2 = sp2 - shift2; // of freezed

				b.conservativeResize(b.rows() + 1);
				b(row_counter) = d2 - d1 + shift1;

				row_counter++;
			}

			else if (model2->getIsFreezed()) // model 1 is the unfreezed one
			{
				size_t id = model_to_par_id_map.at(model1); // id of the unfreezen

				A.conservativeResize(A.rows() + 1, Eigen::NoChange);
				A.row(row_counter ).fill(0);
				A(row_counter, id) = 1;

				float shift1 = model1->getStratigraphicShift(); // of unfreezed
				float shift2 = model2->getStratigraphicShift(); // of freezed

				float d1 = sp1 - shift1; // of unfreezed
				float d2 = sp2 - shift2; // of freezed

				b.conservativeResize(b.rows() + 1);
				b(row_counter) = d2 - d1 + shift2;

				row_counter++;
			}

			else {
				float shift1 = model1->getStratigraphicShift();
				float shift2 = model2->getStratigraphicShift();

				float d1 = sp1 - shift1;
				float d2 = sp2 - shift2;

				size_t id1 = model_to_par_id_map.at(model1);
				size_t id2 = model_to_par_id_map.at(model2);

				A.conservativeResize(A.rows() + 1, Eigen::NoChange);
				A.row(row_counter ).fill(0);
				b.conservativeResize(b.rows() + 1);

				A(row_counter, id1) = 1;
				A(row_counter, id2) = -1;

				b(row_counter) = d2 - d1;

				row_counter++;
			}
		}

		LOG(INFO) << "matrix a \n" << A;

		LOG(INFO) << "vector b \n" << b.transpose();

		VectorXf solution;
		solution.fill(spcNANMacro);

		if (A.rows() == A.cols()) {
			LOG(INFO) << "Problem looks perfectly constrained. Goind to find an exact solution.";
			solution = A.colPivHouseholderQr().solve(b);
		}
		else if (A.rows() < A.cols()) {
			LOG(ERROR) << "Cannot find a solution! The problem is under-constrained \n"
						  "Please consider adding more constrains to your outcrop";
		}
		else if (A.rows() > A.cols()) {
			LOG(WARNING) << "The problem is over-constrained. This might be or not an intended behavior"
							"Consider reducing the number of constrains";

			solution = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
		}

		LOG(INFO) << "solution " << solution.transpose();

		for (int i = 0; i < solution.rows(); ++i) {

			spc::StratigraphicModelBase::Ptr mod = par_id_to_model_map.at(i);
			mod->setStratigraphicShift(solution(i));
		}
	}

	void solve_mixed()
	{
		solve_linear();
		//    solve_linear();
		solve_nonlinear();
	}

    /**
     * @brief models_ containes all the involved models, also freezed models!
     */
    std::vector<StratigraphicModelBase::Ptr> models_;
    std::vector<StratigraphicConstrain::Ptr> constrains_;
    std::vector<StratigraphicPositionableElement::Ptr> positionables_;

    MODE mode_ = MIXED;

    // thi clearly has effect in mixed or non-linear modes
    bool solve_elastic_ = true;
};
}

#endif // STRATIGRAPHICSOLVER_H
