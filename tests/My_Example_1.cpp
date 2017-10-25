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


/*int raising(Eigen::MatrixXf x)
{
 int r = pow(x[0],2);
  return r;
}*/


struct F : public TwiceDifferentiableFunction
{
  
  F(int c) : TwiceDifferentiableFunction(1, 1, "x_3")
  {
    constant = c;
  }
  
  void  impl_compute(result_ref result, const_argument_ref x) const 
  {
   result[0] = pow(x[0],2)+constant;
   
    
  }
  
  void 	impl_gradient (gradient_ref gradient, const_argument_ref x, size_type ) const 
  {
    gradient[0] = 2*x[0];
   // gradient[1] = 2*x[1];
  }
  
  void 	impl_hessian (hessian_ref hessian, const_argument_ref x, size_type ) const 
  {
    hessian(0,0) = 2;
    
  }
  
  int constant;
  
};

/* No Constraints */

int main()
{
    Eigen::MatrixXf X(2,2);
  Eigen::MatrixXf Y(2,1);
   X <<  0, 1,
	1, 1;
  Y << 0,
	1;
  
  int K = 3;
 boost::shared_ptr<F> fun (new F(K));
 
 std::cout << X<<std::endl;
 
 solver_t::problem_t pb(fun);
 
  Function::vector_t start (pb.function ().inputSize ());
  start[0] = 1.;
  
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

 