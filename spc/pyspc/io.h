#ifndef PYSPC_IO_H
#define PYSPC_IO_H


#include "eigen_numpy.h"






#include <spc/core/spc_eigen.h>



#include <spc/methods/RBFModelEstimator.h>

// here is a small python interface for some common functions
namespace spc
{


Eigen::VectorXf getV()
{
	Eigen::VectorXf out(4);

	out << 1,2,3,4;

	return out;
}


class Base
{
public:
	typedef std::shared_ptr<Base> Ptr;

	virtual Ptr clone() const  = 0;

	std::string name_ = "base";

};


class Derived: public Base
{
public:
	using Base::name_;

	typedef std::shared_ptr<Derived> Ptr;

	virtual Base::Ptr clone() const override
	{
		return Base::Ptr(new Derived(*this));
	}

};

void accept_obj(const Base & obj)
{
	std::cout << "received obj as ref" << std::endl;
}


void accept_ptr(const Base::Ptr ptr)
{
	std::cout <<  "received ptr: "<< ptr;
}

Base::Ptr create_ptr()
{
	return Derived::Ptr(new Derived());
}

class TestClass
{
public:
void PrintMat(const Eigen::MatrixXf& m);

};

void PrintMat(const Eigen::MatrixXf& m);



struct World
{

void	operator() () const
{
		LOG(INFO) << "operator";
}

void operator() (const int i) const
{
			LOG(INFO) << "operator II " << i;
}

	void set(std::string msg) { this->msg = msg; }
	std::string greet() { return msg; }
	std::string msg;
};
}

#endif // PYMM_H
