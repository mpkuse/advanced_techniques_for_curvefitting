/// Inputs : [ (x0,y0), (x1,y1), .... ].
// The loss-function is going to be the usual square-error. Which we try to minimize.
///     Will fit a non linear function.

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
    printMatrix1d( "itr"+to_string(summary.iteration)+".txt", params, 3  );
    return ceres::SOLVER_CONTINUE;
  }
private:
  double * params;
};



// Quadratic Least Squares Residue
class LeastSquaresResidueQ {
public:
  LeastSquaresResidueQ( double x, double y ) { this->x = x; this->y = y;}

  template <typename T>
  bool operator()( const T* const params , T* residual ) const {
    T a = params[0];
    T b = params[1];
    T c = params[2];
    residual[0] = y - (a*x*x + b*x + c);
    return true;
  }


  static ceres::CostFunction* Create(double _x, double _y)
  {
    return ( new ceres::AutoDiffCostFunction<LeastSquaresResidueQ,1,3>
      (
        new LeastSquaresResidueQ(_x,_y)
      )
    );
  }

private:
  double x, y;
};


// Quadratic Least Squares Residue with switching constraints. (Is able to identify outliers)
class LeastSquaresResidueQSwitchingConstraint {
public:
  LeastSquaresResidueQSwitchingConstraint( double x, double y ) { this->x = x; this->y = y;}

  template <typename T>
  bool operator()( const T* const params , const T* const s, T* residual ) const {
    T a = params[0];
    T b = params[1];
    T c = params[2];
    residual[0] = s[0] * ( y - (a*x*x + b*x + c) );
    residual[1] = T(10.) * (T(1.0) - s[0]);
    return true;
  }


  static ceres::CostFunction* Create(double _x, double _y)
  {
    return ( new ceres::AutoDiffCostFunction<LeastSquaresResidueQSwitchingConstraint,2,3,1>
      (
        new LeastSquaresResidueQSwitchingConstraint(_x,_y)
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
  generate_quadratic_data( M );

  printEigenMatrix( "M.txt", M);

  //
  // Initial Guess
  double param[3] = {5.,7.,1.0};
  cout << "initial estimates\n";
  printMatrix1d( "init.txt", param, 3 );


  double  * switches = new double[M.rows()];
  for( int i=0 ; i< M.rows() ; i++ ) switches[i] = 1.0;

  //
  // Setup Residue terms
  ceres::Problem problem;
  int NQ = M.rows() ;
  for( int i=0 ; i<NQ ; i++ )
  {
    // CostFunction* cost_function = LeastSquaresResidueQ::Create( M(i,0), M(i,1) );
    // problem.AddResidualBlock( cost_function, NULL, param );
    // problem.AddResidualBlock( cost_function, new ceres::CauchyLoss(.01), param );

    CostFunction* cost_function = LeastSquaresResidueQSwitchingConstraint::Create( M(i,0), M(i,1) );
    problem.AddResidualBlock( cost_function, NULL, param, &switches[i] );
    // problem.AddResidualBlock( cost_function, new ceres::CauchyLoss(.01), param, &switches[i] );
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
  printMatrix1d( "final.txt", param, 3 );

  printMatrix1d( "switches.txt", switches, M.rows() );
  delete [] switches;


}
