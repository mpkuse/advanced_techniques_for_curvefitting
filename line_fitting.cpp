/// Inputs : [ (x0,y0), (x1,y1), .... ].
// The loss-function is going to be the usual square-error. Which we try to minimize.
///     Will fit a linear function.

#include <iostream>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

#include <ceres/ceres.h>
using namespace ceres;

#include "utilities.h"

class ResidueCallback: public ceres::IterationCallback {
public:
  ResidueCallback( double * _params)   {
    params = _params;
  }
  virtual ceres::CallbackReturnType operator()( const ceres::IterationSummary& summary )
  {
    cout << summary.iteration << "   cost=" << summary.cost << endl;
    printMatrix1d( "itr"+to_string(summary.iteration)+".txt", params, 2  );
    return ceres::SOLVER_CONTINUE;
  }
private:
  double * params;
};




class LeastSquaresResidue {
public:
  LeastSquaresResidue( double x, double y ) { this->x = x; this->y = y;}

  template <typename T>
  bool operator()( const T* const params , T* residual ) const {
    T m = params[0];
    T c = params[1];
    residual[0] = y - m*x - c;
    return true;
  }


  static ceres::CostFunction* Create(double _x, double _y)
  {
    return ( new ceres::AutoDiffCostFunction<LeastSquaresResidue,1,2>
      (
        new LeastSquaresResidue(_x,_y)
      )
    );
  }

private:
  double x, y;
};


int main()
{
  //
  // Generate Data
  MatrixXd M;
  generate_linear_data( M );

  printEigenMatrix( "M.txt", M);

  //
  // Initial Guess
  double param[2] = {5.,7.};
  cout << "initial estimates\n";
  printMatrix1d( "init.txt", param, 2 );



  //
  // Setup Residue terms
  ceres::Problem problem;
  for( int i=0 ; i<M.rows() ; i++ )
  {
    CostFunction* cost_function = LeastSquaresResidue::Create( M(i,0), M(i,1) );
    problem.AddResidualBlock( cost_function, new ceres::CauchyLoss(0.1), param );
  }

  //
  // Run
  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;

  // Call back
  ResidueCallback callback(param);
  options.callbacks.push_back(&callback);
  options.update_state_every_iteration = true;

  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  cout << "final_estimates\n";
  printMatrix1d( "final.txt", param, 2 );


}
