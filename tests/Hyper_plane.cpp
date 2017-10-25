  #include <boost/shared_ptr.hpp>

  #include <roboptim/core/linear-function.hh>
  #include <roboptim/core/differentiable-function.hh>
  #include <roboptim/core/twice-differentiable-function.hh>
  #include <roboptim/core/io.hh>
  #include <roboptim/core/solver.hh>
  #include <roboptim/core/solver-factory.hh>
  #include <cmath>
  #include <fstream>
  #include <iostream>





  #define MAXBUFSIZE  ((int) 1e6)


  using namespace roboptim;

  // Specify the solver that will be used.
  typedef Solver<EigenMatrixDense> solver_t;




  ///////////////////////////////////////////////////////
  /* Read file*/

  Function::matrix_t readMatrix(const char *filename, int column_min, int column_max)
      {
      int cols = 0, rows = 0;
      double buff[MAXBUFSIZE];

      // Read numbers from file into buffer.
      std::ifstream infile;
      infile.open(filename);
      while (! infile.eof())
	  {
	  std::string line;
	  getline(infile, line);

	  int temp_cols = 0;
	  std::stringstream stream(line);
      
      // Add all the line
	  while(! stream.eof())
	  {
	      stream >> buff[cols*rows+temp_cols++];
	  }
	

	  if (temp_cols == 0)
	      continue;

	  if (cols == 0)
	  {
	      cols = temp_cols;
	  }

	  rows++;
	  }

      infile.close();

      rows--;

      // Populate matrix with numbers.
      Function::matrix_t result(rows,column_max-column_min+1);
      for (int i = 0; i < rows; i++)
	  for (int j = column_min; j <= column_max; j++)
	      result(i,j-column_min) = buff[ cols*i+j-1 ];

      return result;
      }


  struct F : public TwiceDifferentiableFunction
  {
    
    F(Eigen::MatrixXd X_in, Eigen::VectorXd Y_in) : TwiceDifferentiableFunction(X_in.cols(), 1, "lsq")
    {
  
      X = X_in;
      Y = Y_in;

   vec = 2*Eigen::VectorXd::Identity(X_in.rows(),1);
   
   W = Eigen::MatrixXd::Identity(X_in.rows(),X_in.rows());
   
    }
    
    void impl_compute(result_ref result, const_argument_ref x) const 
    {

 //W = vec.asDiagonal();
    result[0] = ((W*(Y-X*x)).transpose())*(W*(Y-X*x)); 
      
    }
    
   void impl_gradient (gradient_ref gradient, const_argument_ref x, size_type ) const 
    {
      
      gradient = -2*X.transpose()*W.transpose()*W*Y+2*X.transpose()*W.transpose()*W*X*x;
    //  gradient = -2*X.transpose()*Y+2*X.transpose()*X*x;

    }
    
    void impl_hessian (hessian_ref hessian, const_argument_ref x, size_type ) const 
    {
      hessian = 2*W.transpose()*X.transpose()*W*X;
      

    }
        

    matrix_t X;
    vector_t Y;
 Eigen::VectorXd vec;
 Eigen::MatrixXd W;
    
  };

  /* No Constraints */

  
  
  /* =================== Main ============= */
  
  int main()
  {
    
  Function::matrix_t T = Eigen::MatrixXd::Identity(6,6) ;

  double height = 0.05;
  T(3,1) = -height;	T(4,0) = height;


  Function::matrix_t Voltages;
  Eigen::MatrixXd Forces;
  
  Eigen::MatrixXd Shape_mat(6,6);


  const char *file_name = "/home/francesco/Desktop/CALIBRATION/Centauro/sens2.txt";

  Voltages = (readMatrix(file_name, 2, 7)); // mx6
  Forces = (readMatrix(file_name, 8, 13)); // mx6

  Eigen::MatrixXd Forces_sample = Forces*T.transpose(); //mx6


  for (int col_cnt = 0; col_cnt < 6; col_cnt++)
  {
  
  ////////////////////////////////////////////////
  boost::shared_ptr<F> fun (new F(Forces_sample, Voltages.col(col_cnt)));

  solver_t::problem_t pb(fun);
  
    Function::vector_t start (pb.function ().inputSize ());
    start = Eigen::MatrixXd::Zero(6,1);
    
    pb.startingPoint() = start;
    
    SolverFactory<solver_t> factory ("ipopt", pb);
    solver_t& solver = factory ();
    
    solver_t::result_t res = solver.minimum ();

   /* Result& result = boost::get<Result> (res);
    Shape_mat.row(col_cnt) = result.x;
    
    std::cout <<Shape_mat<<std::endl;*/
    
    switch (res.which ())
    {
    case solver_t::SOLVER_VALUE:
      {
        // Get the result.
        Result& result = boost::get<Result> (res);

        // Display the result.
        std::cout << "A solution has been found: " << std::endl
                  << result << std::endl;
		  	
	 /*std::cout << "HETYYYYYYYYYYYYYYYYYYYYY"<< std::endl;    
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
    
  }
    return 0;
 
  }


  