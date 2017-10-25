#include <boost/shared_ptr.hpp>

#include <roboptim/core/linear-function.hh>
#include <roboptim/core/differentiable-function.hh>
#include <roboptim/core/twice-differentiable-function.hh>
#include <roboptim/core/io.hh>
#include <roboptim/core/solver.hh>
#include <roboptim/core/solver-factory.hh>
#include <cmath>

using namespace roboptim;

// Specify the solver that will be used.
typedef Solver<EigenMatrixDense> solver_t;





class F : public DifferentiableFunction
{
public:
  F() : DifferentiableFunction(2, 1, "lsq")
  {
    //err = matrix_t(2,1);
  /*  X = Eigen::MatrixXd::Zero(2,2);
    Y = Eigen::VectorXd::Zero(2);
    X(0,0) = 0;	X(0,1) = 1;
    X(1,0) = 1;	X(1,1) = 1;
      
    Y(0) = 0;	Y(1) = 1; */
 

   // err = Eigen::VectorXd::Zero(2);
  }
  
  
  void  impl_compute(result_ref result, const_argument_ref x) const ;
  /*{
    

     // Eigen::VectorXd err = Y-X*x;
     
   result[0] = Err_Norm(x).transpose()*Err_Norm(x); 
    
  }*/
  
  void 	impl_gradient (gradient_ref gradient, const_argument_ref x, size_type ) const; 
  /*{
   // gradient = -2*X.transpose()*Y+2*X.transpose()*X*x;
    gradient = -2*X*Err_Norm(x);
   // gradient[0] = 2*x[0];
    //gradient[1] = 2*x[1];
  }*/
  
 /* void 	impl_hessian (hessian_ref hessian, const_argument_ref x, size_type ) const 
  {
    hessian = 2*X.transpose()*X;

    
  }*/
 


  static Eigen::MatrixXd X;
  static Eigen::VectorXd Y;
  static Eigen::VectorXd err;
  static double norm;
  
};

  Eigen::VectorXd Err_Norm( Eigen::VectorXd x)
{
 /* Eigen::MatrixXd X = Eigen::MatrixXd::Zero(2,2);
     Eigen::VectorXd Y = Eigen::VectorXd::Zero(2);
    X(0,0) = 0;	X(0,1) = 1;
    X(1,0) = 1;	X(1,1) = 1;
      
    Y(0) = 0;	Y(1) = 1; */
    
   
  
        Eigen::VectorXd err = F::Y-F::X*x;
     
 // double norm = (err.transpose())*err;

 // norm = err.squaredNorm();
 //norm = sqrt(norm);
 
 return err;
}


void F::impl_compute(result_ref result, const_argument_ref x) const 
{
  
    result[0] = Err_Norm(x).transpose()*Err_Norm(x); 
  
}

void F::impl_gradient (gradient_ref gradient, const_argument_ref x, size_type ) const 
{
  gradient = -2*X*Err_Norm(x);
}

/* No Constraints */
 
 Eigen::MatrixXd F::X = Eigen::MatrixXd::Zero(2,2);
 Eigen::VectorXd F::Y = Eigen::VectorXd::Zero(2);
 
 
int main()
{
  
boost::shared_ptr<F> fun (new F());

 
   F::X(0,0) = 0;	F::X(0,1) = 1;
    F::X(1,0) = 1;	F::X(1,1) = 1;
      
    F::Y(0) = 0;	F::Y(1) = 1; 
 
 
 
 solver_t::problem_t pb(fun);
 
  Function::vector_t start (pb.function ().inputSize ());
  start[0] = 1.; start[1] = 1.;
  
  pb.startingPoint() = start;
  
   SolverFactory<solver_t> factory ("ipopt", pb);
  solver_t& solver = factory ();
  
  solver_t::result_t res = solver.minimum ();
  
   std::cout << solver << std::endl;
   
   switch (res.which ())
    {
    case solver_t::SOLVER_VALUE:
      {
        // Get the result.
        Result& result = boost::get<Result> (res);

        // Display the result.
        std::cout << "A solution has been found: " << std::endl
                  << result << std::endl;
		  	
	/* std::cout << "HETYYYYYYYYYYYYYYYYYYYYY"<< std::endl;    
    std::cout << fun->Y-(fun->X*result.x)<< std::endl;
     std::cout << err<< std::endl;*/
    
    
    
        return 0;

	
	
      }

    case solver_t::SOLVER_ERROR:
      {
        std::cout << "A solution should have been found. Failing..."
                  << std::endl
                  << boost::get<SolverError> (res).what ()
                  << std::endl;

        return 2;
      }

    case solver_t::SOLVER_NO_SOLUTION:
      {
        std::cout << "The problem has not been solved yet."
                  << std::endl;

        return 2;
      }
    }


    
    
  // Should never happen.
  assert (0);
  return 42;

	 
	 
}

 