// Input is 2 sets of pointclouds. TO estimate rotation + translation between the two.
// SO(3) + R^3.
// Also uses http://ceres-solver.org/nnls_covariance.html (Covariance from Jacobians at final value)

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



class EuclideanDistanceResidueSwitchingConstraint  {
public:
  EuclideanDistanceResidueSwitchingConstraint( const Vector3d& Xi, const Vector3d& Xid )
  {
    this->Xi = Xi;
    this->Xid = Xid;
  }

  template <typename T>
  bool operator()( const T* const q, const T* const t , const T* const s, T* residual ) const {
    // Optimization variables
    Quaternion<T> eigen_q( q[0], q[1], q[2], q[3] );
    Eigen::Matrix<T,3,1> eigen_t;
    eigen_t << t[0], t[1], t[2];


    // Known Constant
    Eigen::Matrix<T,3,1> eigen_Xi, eigen_Xid;
    eigen_Xi << T(Xi(0)), T(Xi(1)), T(Xi(2));
    eigen_Xid << T(Xid(0)), T(Xid(1)), T(Xid(2));



    // Error term

    Eigen::Matrix<T,3,1> e;
    e = eigen_Xi - (  eigen_q.toRotationMatrix() * eigen_Xid + eigen_t );

    residual[0] = s[0] * e(0);
    residual[1] = s[0] * e(1);
    residual[2] = s[0] * e(2);
    residual[3] = T(5.) * (T(1.0) - s[0]);

    return true;
  }



  static ceres::CostFunction* Create(const Vector3d& _Xi, const Vector3d& Xid)
  {
    return ( new ceres::AutoDiffCostFunction<EuclideanDistanceResidueSwitchingConstraint,4,   4,3,1>
      (
        new EuclideanDistanceResidueSwitchingConstraint(_Xi,Xid)
      )
    );
  }

private:
  Vector3d Xi, Xid;
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
    eigen_t << t[0], t[1], t[2];


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


void A( MatrixXd& w_X, MatrixXd& w_Xd )
{
  MatrixXd M;
  load_pointcloud( "../data3d/mannequin_mini.dat", 4, M );
  int npts = M.rows();

  w_X = M.leftCols(3).transpose();

  MatrixXd neta = MatrixXd::Random(w_X.rows(), w_X.cols()) * 10;
  w_Xd = ypr2R( Vector3d(80, 0, 0) ) * w_X  +  neta  ;
}

// Return w_X and w_Xd in homogeneous co-ordinates
void B( MatrixXd& w_X, MatrixXd& w_Xd )
{
  MatrixXd M;
  load_pointcloud( "../data3d/mannequin_mini.dat", 4, M );
  int npts = M.rows();

  w_X = MatrixXd( 4, npts );
  w_X <<  M.leftCols(3).transpose(), MatrixXd::Ones(1,npts);

  MatrixXd neta = MatrixXd::Random(w_X.rows(), w_X.cols()) * 0.;
  Matrix4d T;
  // T << ypr2R( Vector3d(0, 0, 0) ), Vector3d(200,41.6,122), MatrixXd::Zero(1,3), 1.0; // pure translation
  // T << ypr2R( Vector3d(-30, 120, 90) ), Vector3d(0,0,0), MatrixXd::Zero(1,3), 1.0; // pure rotation
  T << ypr2R( Vector3d(-30, 120, 90) ), Vector3d(200,41.6,122), MatrixXd::Zero(1,3), 1.0;
  printPoseMatrix( T );
  cout << "___________T:\n"<< T << endl;
  printPoseMatrix( T.inverse() );
  cout << "___________Tinv:\n"<< T.inverse() << endl;
  w_Xd = T * w_X  + neta ;
}

// extra random outliers
void C( MatrixXd& w_Y, MatrixXd& w_Yd )
{
  MatrixXd M;
  load_pointcloud( "../data3d/mannequin_mini.dat", 4, M );
  int npts = M.rows();

  MatrixXd w_X, w_Xd;
  w_X = MatrixXd( 4, npts );
  w_X <<  M.leftCols(3).transpose(), MatrixXd::Ones(1,npts);

  MatrixXd neta = MatrixXd::Random(w_X.rows(), w_X.cols()) * 20;
  Matrix4d T;
  // T << ypr2R( Vector3d(0, 0, 0) ), Vector3d(200,41.6,122), MatrixXd::Zero(1,3), 1.0; // pure translation
  // T << ypr2R( Vector3d(-30, 120, 90) ), Vector3d(0,0,0), MatrixXd::Zero(1,3), 1.0; // pure rotation
  T << ypr2R( Vector3d(-30, 120, 90) ), Vector3d(200,41.6,122), MatrixXd::Zero(1,3), 1.0;
  printPoseMatrix( T );
  cout << "___________T:\n"<< T << endl;
  printPoseMatrix( T.inverse() );
  cout << "___________Tinv:\n"<< T.inverse() << endl;
  w_Xd = T * w_X  + neta;



  // Insane points
  int n_insane_pts = 500;
  MatrixXd w_Z = MatrixXd( 4, n_insane_pts);
  w_Z << MatrixXd::Random( 3, n_insane_pts )*500,  MatrixXd::Ones(1,n_insane_pts);

  MatrixXd w_Zd = MatrixXd( 4, n_insane_pts);
  w_Zd << MatrixXd::Random( 3, n_insane_pts )*500,  MatrixXd::Ones(1,n_insane_pts);


  // combine sane points with insane points
  // w_Y = Union( w_X,  w_Z )
  // w_Yd = Union( w_Xd,  w_Zd )
  w_Y = MatrixXd( 4, npts + n_insane_pts );
  w_Y << w_X, w_Z;
  w_Yd = MatrixXd( 4, npts + n_insane_pts );
  w_Yd << w_Xd, w_Zd;


}

void realA(  MatrixXd& w_X, MatrixXd& w_Xd  )
{
    string _1 = "mateigen_86_51___w_X_iprev_triangulated.txt";
    string _2 = "mateigen_86_51___w_X_icurr_triangulated.txt";

    MatrixXd M, M1;
    load_pointcloud( "../data3d/"+_1, 4, M );
    load_pointcloud( "../data3d/"+_2, 4, M1 );

    w_X = M.transpose();
    w_Xd = M1.transpose();
}


int main()
{

  MatrixXd w_X, w_Xd;
  // A(w_X, w_Xd );
  B(w_X, w_Xd );
  // C(w_X, w_Xd );

  // realA( w_X, w_Xd );

  printEigenMatrix( "w_X.txt", w_X );
  printEigenMatrix( "w_Xd.txt", w_Xd );
  printMatrixInfo( "w_X", w_X );
  printMatrixInfo( "w_Xd", w_Xd );



  //
  // Initial Guess
  Matrix4d T_cap = Matrix4d::Identity();
  double T_cap_q[10], T_cap_t[10]; //quaternion and translation
  eigenmat_to_raw( T_cap, T_cap_q, T_cap_t );
  cout << " Initial Pose Estimate:\n" << endl;
  printPoseMatrix( T_cap );
  printEigenMatrix( T_cap, "T_cap_init" );
  printEigenMatrix( "T_cap_init.txt", T_cap  );

  double * s = new double [w_X.cols()];
  for( int i=0; i<w_X.cols() ; i++ ) s[i] = 1.0;


  //
  // Setup Residue terms
  ceres::Problem problem;
  cout << "# Residue terms = " << w_X.cols() << endl;
  for( int i=0 ; i<w_X.cols() ; i++ )
  {
    CostFunction* cost_function = EuclideanDistanceResidue::Create( w_X.col(i).head(3), w_Xd.col(i).head(3) );
    problem.AddResidualBlock( cost_function, NULL, T_cap_q, T_cap_t );
    // problem.AddResidualBlock( cost_function, new CauchyLoss(.01), T_cap_q, T_cap_t );



    // CostFunction* cost_function = EuclideanDistanceResidueSwitchingConstraint::Create( w_X.col(i).head(3), w_Xd.col(i).head(3) );
    // problem.AddResidualBlock( cost_function, NULL, T_cap_q, T_cap_t, &s[i] );
    // problem.AddResidualBlock( cost_function, new CauchyLoss(.1), T_cap_q, T_cap_t, &s[i] );
  }

  // Local Parameterization (for 6DOF)
  ceres::LocalParameterization *quaternion_parameterization = new ceres::QuaternionParameterization;
  problem.SetParameterization( T_cap_q, quaternion_parameterization );


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
  std::cout << summary.FullReport() << "\n";

  //
  // Retrive Final Estimate of pose
  raw_to_eigenmat( T_cap_q, T_cap_t, T_cap );
  cout << " Final Pose Estimate:\n" << endl;
  printPoseMatrix( T_cap );
  printEigenMatrix( T_cap, "T_cap_final" );
  printEigenMatrix( "T_cap_final.txt", T_cap  );
  printMatrix1d( "switches.txt", s, w_X.cols()  );

}
