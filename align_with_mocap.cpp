// Given a set of poses (of origin) in co-ordinates ref of vins P: vector<Matrix4d>
// and poses in co-ordinate ref of mocap Q: vector<Matrix4d>. To get the relative transform.


#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <iterator>

using namespace std;



#include "utilities.h"

#define OPT_TYPE 1


// Solves: minimize{vins_T_vicon} || vins_T_c - vins_T_vicon * vicon_T_c ||_2^2
// Given 2 sets of poses to find a relative transform between the 2 co-ordinate system.
// a_T_i is the pose of i^{th} image in frame-of-ref of a
// b_T_i is the pose of i^{th} image in frame-of-ref of b
class AlignPoses {
public:
  AlignPoses( const Matrix4d& a_T_i, const Matrix4d& b_T_i, const double _weight=1.0 )
          :a_T_i(a_T_i), b_T_i(b_T_i), weight( _weight )
          {
          }

          #if OPT_TYPE == 1
  ///////////////////////////// 1 /////////////////////////////////
  template <typename T>
  bool operator()( const T* const quat, const T* const tran, T*e  ) const
  {
      Eigen::Quaternion<T> q( quat[0], quat[1], quat[2], quat[3] );//w,x,y,z
      Eigen::Matrix<T,3,1> t;
      t<< tran[0], tran[1], tran[2];

      Eigen::Matrix<T,4,4> X; //the transform, made up of q, t
      X << q.toRotationMatrix() , t , T(0.0), T(0.0), T(0.0), T(1.0);


    Eigen::Matrix<T,4,4> A, B;
    A = a_T_i.cast<T>();
    B = b_T_i.cast<T>();

    Eigen::Matrix<T,4,4> err = (A * X) * (X*B).inverse();
    Eigen::Matrix<T,3,3> err_R = err.topLeftCorner(3,3);
    Eigen::Quaternion<T> err_quat( err_R );
    e[0] = T(1.01) * err_quat.x();
    e[1] = T(1.01) * err_quat.y();
    e[2] = T(1.01) * err_quat.z();
    e[3] = err(0,3);
    e[4] = err(1,3);
    e[5] = err(2,3);
    return true;

  }

  static ceres::CostFunction* Create( const Matrix4d& a_T_i, const Matrix4d& b_T_i, const double weight=1.0 )
  {
    return (
       new ceres::AutoDiffCostFunction<AlignPoses,6,4,3>( new AlignPoses( a_T_i, b_T_i, weight ) )
    );
  }
  //////////////////////////////////////////////////////////////////
  #endif



  #if OPT_TYPE == 2
  ///////////////////////////// 2 /////////////////////////////////
  template <typename T>
  bool operator()( const T* const quat, const T* const tran, const T* const s, T*e  ) const
  {
      T logisitc = T(1.0) / ( T(1.0) + ceres::exp( T(-4.0) * T(*s) ) );
      Eigen::Quaternion<T> q( quat[0], quat[1], quat[2], quat[3] );//w,x,y,z
      Eigen::Matrix<T,3,1> t;
      t<< tran[0], tran[1], tran[2];

      Eigen::Matrix<T,4,4> X; //the transform, made up of q, t
      X << q.toRotationMatrix() , t , T(0.0), T(0.0), T(0.0), T(1.0);


    Eigen::Matrix<T,4,4> A, B;
    A = a_T_i.cast<T>();
    B = b_T_i.cast<T>();

    Eigen::Matrix<T,4,4> err = (A * X) * (X*B).inverse();
    Eigen::Matrix<T,3,3> err_R = err.topLeftCorner(3,3);
    Eigen::Quaternion<T> err_quat( err_R );
    e[0] = logisitc * T(1.01) * err_quat.x();
    e[1] = logisitc * T(1.01) * err_quat.y();
    e[2] = logisitc * T(1.01) * err_quat.z();
    e[3] = logisitc * err(0,3);
    e[4] = logisitc * err(1,3);
    e[5] = logisitc * err(2,3);
    e[6] = T( 0.1 ) * ( T(1.) - (*s) );
    return true;

  }

  static ceres::CostFunction* Create( const Matrix4d& a_T_i, const Matrix4d& b_T_i, const double weight=1.0 )
  {
    return (
       new ceres::AutoDiffCostFunction<AlignPoses,7,4,3,1>( new AlignPoses( a_T_i, b_T_i, weight ) )
    );
  }
  //////////////////////////////////////////////////////////////////
  #endif


private:
  Matrix4d a_T_i; //3d point in world co-ordinate
  Matrix4d b_T_i; //undistorrted normalized observed points
  double weight;

};




int main(int argc, char** argv) {

  double x = 0.5;
  const double initial_x = x;

  // Load Data
  vector<Matrix4d> P, Q;

  loadtxt( "../data_poses/P.txt", P );
  loadtxt( "../data_poses/Q.txt", Q );


  // Plotting angles
  ofstream myfileP( "../tmp/P_ypr.txt");
  for( int i=0 ; i<P.size() ; i++ )
  {
      double ypr[5], t[5];
      eigenmat_to_rawyprt( P[i], ypr, t );
      myfileP << ypr[0] << "," << ypr[1] << "," << ypr[2] << endl;
  }
  myfileP.close();

  ofstream myfileQ( "../tmp/Q_ypr.txt");
  for( int i=0 ; i<Q.size() ; i++ )
  {
      double ypr[5], t[5];
      eigenmat_to_rawyprt( Q[i], ypr, t );
      myfileQ << ypr[0] << "," << ypr[1] << "," << ypr[2] << endl;
  }
  myfileQ.close();

  // Done writing ypr data to file

    //
    // Optimization variable
    Matrix4d X = Matrix4d::Identity();
    X << -0.883672, -0.0985627,  -0.457612,  -0.167047,
  0.43597,   0.182674,  -0.881227,   0.112866,
  0.17045,  -0.978221,  -0.118453,    -0.1798,
        0,          0,          0,          1;

    double X_quat[5], X_t[5];
    eigenmat_to_raw( X, X_quat, X_t );

    #if OPT_TYPE == 2
    double s[1000] = {0.98};
    #endif

    //
    // Setup Residues
    ceres::Problem problem;
    int nResidues = 0;
    for( int i=20 ; i<80 /*P.size()*/ ; i++ )
    {
        for( int j=max(0,i-10) ; j<i ; j++ ) {
            // cout << i << " " << i-1-j << endl;
        #if OPT_TYPE == 1
        ceres::CostFunction * cost_function = AlignPoses::Create( Q[i].inverse() * Q[i-1-j], P[i].inverse() * P[i-1-j], 1.0 );
        // problem.AddResidualBlock( cost_function,  new ceres::CauchyLoss( 0.1 ), p_quat_q, p_t_q );
        problem.AddResidualBlock( cost_function,  NULL, X_quat, X_t );
        #endif

        #if OPT_TYPE == 2
        ceres::CostFunction * cost_function = AlignPoses::Create( Q[i].inverse() * Q[i-1-j], P[i].inverse() * P[i-1-j], 1.0 );
        problem.AddResidualBlock( cost_function,  NULL, X_quat, X_t, &s[nResidues] );
        #endif

        nResidues++;
        }
    }
    cout << "nResidues = " << nResidues << endl;

    ceres::LocalParameterization * quat_parameterization = new ceres::QuaternionParameterization;
    problem.SetParameterization( X_quat, quat_parameterization );

    //
    // Run
    ceres::Solver::Options options;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.dogleg_type = ceres::SUBSPACE_DOGLEG;
    options.use_nonmonotonic_steps = true;
    options.function_tolerance = 1e-8;
    // options.preconditioner_type = ceres::CLUSTER_TRIDIAGONAL;
    options.use_inner_iterations = true;
    // options.minimizer_type = ceres::LINE_SEARCH;
    // options.line_search_direction_type = ceres::BFGS;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    ceres::Solve( options, &problem, &summary );
    cout << summary.BriefReport() << endl;


    //
    // Retrive Results
    raw_to_eigenmat( X_quat, X_t, X );


    cout << "X:\n" << X << endl; // X : h_T_c
    Matrix4d w_T_wd = Q[10] * X * P[10].inverse();
    Matrix4d w_T_wd__2 =  Q[10] * X * P[10].inverse();
    cout << "w_T_wd:"; printPoseMatrix( w_T_wd );
    cout << "\n" << w_T_wd << endl;
    cout << "w_T_wd__2:"; printPoseMatrix( w_T_wd__2 );
    cout << w_T_wd__2 << endl;


    cout << "rel << ";
    for( int i=0; i<4; i++ )
    {
        for (int j=0 ; j<4; j++ )
        {
            cout << w_T_wd__2(i,j) << ",";
        }
        cout << endl;
    }
    cout << endl;


    for( int i=0 ; i<P.size() ; i++ )
    {
        cout << "--- " << i << " ---\n";
        // cout << "P:\n" << P[i] << endl;
        cout << "P:"; printPoseMatrix( P[i] );


        // cout << "Q:\n" << X * Q[i] << endl;
        Matrix4d Q_mod;
        Q_mod = w_T_wd.inverse() * Q[i];
        cout << "Q:"; printPoseMatrix( Q_mod );


    }

    #if OPT_TYPE == 2
    for( int i=0 ; i < nResidues ; i++ )
    {
        cout << i << ":" << s[i] << "\t";
    }
    #endif


  return 0;
}
