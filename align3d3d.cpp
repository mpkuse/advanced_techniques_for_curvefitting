// Input is 2 sets of pointclouds. TO estimate rotation + translation between the two.
// SO(3) + R^3.

#include "utilities.h"


class EuclideanDistanceResidueCallback: public ceres::IterationCallback {
public:
  EuclideanDistanceResidueCallback( double * _params_q, double * _params_t )   {
    params_q = _params_q;
    params_t = _params_t;
  }

  virtual ceres::CallbackReturnType operator()( const ceres::IterationSummary& summary )
  {
    cout << "---\n";
    cout << summary.iteration << "   cost=" << summary.cost << endl;
    // printMatrix1d( "itr"+to_string(summary.iteration)+".txt", params, 2  );

    Matrix4d T_cap;
    raw_to_eigenmat( params_q, params_t, T_cap );
    printEigenMatrix( "T_cap_"+to_string(summary.iteration)+".txt", T_cap );
    // printEigenMatrix( T_cap, "T_cap_"+to_string(summary.iteration)+".txt"  );
    printPoseMatrix( T_cap );
    return ceres::SOLVER_CONTINUE;
  }
private:
  double * params_q, * params_t;
};



class EuclideanDistanceResidue {
public:
  EuclideanDistanceResidue( const Vector3d& Xi, const Vector3d& Xid )
  {
    this->Xi = Xi;
    this->Xid = Xid;
  }

  template <typename T>
  bool operator()( const T* const q, const T* const t , T* residual ) const {
    // Optimization variables
    Quaternion<T> eigen_q( q[0], q[1], q[2], q[3] );
    Eigen::Matrix<T,3,1> eigen_t;
    eigen_t << t[0], t[1], t[1];


    // Known Constant
    Eigen::Matrix<T,3,1> eigen_Xi, eigen_Xid;
    eigen_Xi << T(Xi(0)), T(Xi(1)), T(Xi(2));
    eigen_Xid << T(Xid(0)), T(Xid(1)), T(Xid(2));



    // Error term

    Eigen::Matrix<T,3,1> e;
    e = eigen_Xi - (  eigen_q.toRotationMatrix() * eigen_Xid + eigen_t );

    residual[0] = e(0);
    residual[1] = e(1);
    residual[2] = e(2);

    return true;
  }



  static ceres::CostFunction* Create(const Vector3d& _Xi, const Vector3d& Xid)
  {
    return ( new ceres::AutoDiffCostFunction<EuclideanDistanceResidue,3,4,3>
      (
        new EuclideanDistanceResidue(_Xi,Xid)
      )
    );
  }

private:
  Vector3d Xi, Xid;
};


int main()
{
  //
  // Generate 2 point clouds
  MatrixXd M;
  load_pointcloud( "../data3d/mannequin_mini.dat", 4, M );

  MatrixXd w_X = M.leftCols(3).transpose();
  printMatrixInfo( "w_X", w_X ); // w_X is 3xN

  MatrixXd neta = MatrixXd::Random(w_X.rows(), w_X.cols()) * 10;
  MatrixXd w_Xd = ypr2R( Vector3d(80, 0, 0) ) * w_X   +  neta  ;

  printEigenMatrix( "w_X.txt", w_X );
  printEigenMatrix( "w_Xd.txt", w_Xd );



  //
  // Initial Guess
  Matrix4d T_cap = Matrix4d::Identity();
  double T_cap_q[10], T_cap_t[10]; //quaternion and translation
  eigenmat_to_raw( T_cap, T_cap_q, T_cap_t );
  printEigenMatrix( T_cap, "T_cap_init" );
  printEigenMatrix( "T_cap_init.txt", T_cap  );


  //
  // Setup Residue terms
  ceres::Problem problem;
  for( int i=0 ; i<w_X.cols() ; i++ )
  {
    CostFunction* cost_function = EuclideanDistanceResidue::Create( w_X.col(i), w_Xd.col(i) );
    problem.AddResidualBlock( cost_function, NULL, T_cap_q, T_cap_t );
  }


  //
  // Run
  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;

  // CallbackReturnType
  EuclideanDistanceResidueCallback callback(T_cap_q, T_cap_t);
  options.callbacks.push_back(&callback);
  options.update_state_every_iteration = true;

  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  //
  // Retrive Final Estimate of pose
  raw_to_eigenmat( T_cap_q, T_cap_t, T_cap );
  cout << "ypr" << R2ypr( T_cap.topLeftCorner<3,3>() ) << endl;
  printEigenMatrix( T_cap, "T_cap_final" );
  printEigenMatrix( "T_cap_final.txt", T_cap  );

}
