#pragma once

#include <iostream>
#include <fstream>
#include <math.h>
#include <random>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#include <ceres/ceres.h>
using namespace ceres;

//////////////////////////// IO Utils ////////////////////////////////////////
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

#define print_output_file_info( msg ) msg;

template <typename Derived>
void printEigenMatrix(const string& filename, const MatrixBase<Derived>& a)
{
  std::ofstream file(filename);
  if( file.is_open() )
  {
    file << a.format(CSVFormat) << endl;
    print_output_file_info(cout << "\033[1;32m" <<"Written to file: "<< filename  << "\033[0m\n" );
  }
  else
  {
    cout << "\033[1;31m" << "FAIL TO OPEN FILE for writing: "<< filename << "\033[0m\n";

  }
}


void printMatrix2d( const string& filename, const double * D, int nRows, int nCols )
{
  std::ofstream file(filename);
  if( file.is_open() )
  {
    int c = 0 ;
    for( int i=0; i<nRows ; i++ )
    {
      file << D[c];
      c++;
      for( int j=1 ; j<nCols ; j++ )
      {
        file << ", " << D[c] ;
        c++;
      }
      file << "\n";
    }
    print_output_file_info( cout << "\033[1;32m" <<"Written to file: "<< filename  << "\033[0m\n" );
  }
  else
  {
    cout << "\033[1;31m" << "FAIL TO OPEN FILE for writing: "<< filename << "\033[0m\n";

  }

}

void printMatrix1d( const string& filename, const double * D, int n  )
{
  std::ofstream file(filename);
  if( file.is_open() )
  {
    file << D[0];
    for( int i=1 ; i<n ; i++ )
      file << ", " << D[i] ;
    file << "\n";
    print_output_file_info(cout << "\033[1;32m" <<"Written to file: "<< filename  << "\033[0m\n");
  }
  else
  {
    cout << "\033[1;31m" << "FAIL TO OPEN FILE for writing: "<< filename << "\033[0m\n";

  }

}

template <typename Derived>
void printEigenMatrix(const MatrixBase<Derived>& a, const string& msg )
{
  cout << msg << endl;
  cout << a.format(CSVFormat) << endl;

}


void printMatrix2d( const double * D, int nRows, int nCols, const string& msg  )
{
  cout << msg << endl;
    int c = 0 ;
    for( int i=0; i<nRows ; i++ )
    {
      cout << D[c];
      c++;
      for( int j=1 ; j<nCols ; j++ )
      {
        cout << ", " << D[c] ;
        c++;
      }
      cout << "\n";
    }


}

void printMatrix1d( const double * D, int n, const string& msg   )
{
  cout << msg << endl;
    cout << D[0];
    for( int i=1 ; i<n ; i++ )
      cout << ", " << D[i] ;
    cout << "\n";

}

void printMatrixInfo( const MatrixXd& M, const string& msg ) {
  cout << msg << ":" << "rows=" << M.rows() << ", cols=" << M.cols() << endl;
}
void printMatrixInfo( const string& msg, const MatrixXd& M ) {
  cout << msg << ":" << "rows=" << M.rows() << ", cols=" << M.cols() << endl;
}




void generate_linear_data(  MatrixXd& D )
{
  int N = 100;
  D = MatrixXd::Zero(N, 2);

  std::default_random_engine generator;
  std::normal_distribution<double> distribution_normal(.0,1.0);
  std::uniform_real_distribution<double> distribution_uniform(-10.0,10.0);


  for( int i=0 ; i<N ; i++ )
  {
    D(i,0) = distribution_uniform( generator );
    D(i,1) =  2.0 * D(i,0) + 6.0 + distribution_normal( generator );

    if( i > 30 )
    {
      D(i,0) = 100*distribution_uniform( generator );
      D(i,1) = distribution_uniform( generator );
    }
  }

}


void generate_quadratic_data(  MatrixXd& D )
{
  int N = 100;
  D = MatrixXd::Zero(N, 2);

  std::default_random_engine generator;
  std::normal_distribution<double> distribution_normal(.0,1.0);
  std::uniform_real_distribution<double> distribution_uniform(-10.0,10.0);


  for( int i=0 ; i<N ; i++ )
  {
    D(i,0) = distribution_normal( generator ) * 10;
    D(i,1) =  2.0 * D(i,0) * D(i,0) + 6.0 * D(i,0) + 7. + distribution_normal( generator );

    if( i > 70 )
    {
      D(i,0) = 10*distribution_uniform( generator );
      D(i,1) = 25*distribution_uniform( generator );
    }
  }

}



void load_pointcloud( const string& filename, int number_per_line, MatrixXd& M )
{
  std::ifstream file(filename);

  if( file.is_open() )
  {
    cout << "\033[1;32m" <<"Opened file: "<< filename  << "\033[0m\n";
    std::vector<double> array;
    double number;
    while(file >> number) {
      array.push_back(number);
    }
    cout << "from file " << filename <<  "# numbers read: "<< array.size() << endl;
    assert( array.size()%number_per_line == 0 );
    M = MatrixXd::Zero(  array.size()/number_per_line , number_per_line );
    for( int i=0 ; i<(array.size()/number_per_line) ; i++ )
    {
      for(int j=0 ; j<number_per_line ; j++ )
      {
        M( i, j ) = array[ number_per_line * i + j ];
      }
    }
  }
  else
  {
    cout << "\033[1;31m" << "FAIL TO OPEN FILE for reading: "<< filename << "\033[0m\n";
  }
}




template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

void printPoseMatrix( const Matrix4d& M );

// in the test file the matrix is in row major format, and occupies 4 lines per matrix.
// number of lines in the file need to be a multiple of 4.
void loadtxt( const string& fname, vector<Matrix4d>& A  )
{
    //
    // Read txt file
    cout << "Open file: "<< fname << endl;
    std::ifstream file( fname );
    vector<string> vec_of_string;
    string str;
    int nline=0;
    while( getline(file, str ) )
    {
        // cout << str << endl;
        nline++;
        vec_of_string.push_back( str );
    }
    cout << "Done reading txt " << nline << endl;
    assert( nline%4 == 0 );



    //
    // Go thru the text file and set Matrix4d s.
    for( int i=0 ; i<nline ; i+=4 )
    {
        Matrix4d M;
        for( int j=0 ; j<4 ; j++ )
        {
            vector<string> splitted_row_j = split( vec_of_string[i+j], ',' );
            assert( splitted_row_j.size() == 4 );

            M( j, 0 ) = stod( splitted_row_j[0] );
            M( j, 1 ) = stod( splitted_row_j[1] );
            M( j, 2 ) = stod( splitted_row_j[2] );
            M( j, 3 ) = stod( splitted_row_j[3] );
        }
        A.push_back( M );
    }


    //
    //
    cout << A.size() << " matrixces in the file\n";
    for( int i=0 ; i<A.size() ; i++ )
    {
        cout << fname << "[" << i << "]=\n";
        printPoseMatrix( A[i] );
        cout << A[i] << "\n\n";

    }


}



//////////////////////////// END IO Utils ////////////////////////////////////////




///////////////////////////// Pose Utils /////////////////////////////////////

Vector3d R2ypr( const Matrix3d& R)
{
  Eigen::Vector3d n = R.col(0);
  Eigen::Vector3d o = R.col(1);
  Eigen::Vector3d a = R.col(2);

  Eigen::Vector3d ypr(3);
  double y = std::atan2(n(1), n(0));
  double p = std::atan2(-n(2), n(0) * std::cos(y) + n(1) * std::sin(y));
  double r = std::atan2(a(0) * std::sin(y) - a(1) * std::cos(y), -o(0) * std::sin(y) + o(1) * std::cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / M_PI * 180.0;
}


Matrix3d ypr2R( const Vector3d& ypr)
{
  double y = ypr(0) / 180.0 * M_PI;
  double p = ypr(1) / 180.0 * M_PI;
  double r = ypr(2) / 180.0 * M_PI;



  // Eigen::Matrix<double, 3, 3> Rz;
  Matrix3d Rz;
  Rz << std::cos(y), -std::sin(y), 0,
      std::sin(y), std::cos(y), 0,
      0, 0, 1;


  // Eigen::Matrix<double, 3, 3> Ry;
  Matrix3d Ry;
  Ry << std::cos(p), 0., std::sin(p),
      0., 1., 0.,
      -std::sin(p), 0., std::cos(p);

  // Eigen::Matrix<double, 3, 3> Rx;
  Matrix3d Rx;
  Rx << 1., 0., 0.,
      0., std::cos(r), -std::sin(r),
      0., std::sin(r), std::cos(r);

  return Rz * Ry * Rx;

}

Matrix3d ypr2R( double yy, double pp, double rr )
{
    Vector3d ypr;
    ypr << yy, pp, rr;
    return ypr2R( ypr );
}

void printPoseMatrix( const Matrix4d& M )
{
  cout << "\tYPR      : " << R2ypr(  M.topLeftCorner<3,3>() ).transpose() << "  ";
  cout << "\tTx,Ty,Tz : " << M(0,3) << ", " << M(1,3) << ", " << M(2,3) << endl;
}

void raw_to_eigenmat( const double * quat, const double * t, Matrix4d& dstT )
{
  Quaterniond q = Quaterniond( quat[0], quat[1], quat[2], quat[3] );

  dstT = Matrix4d::Zero();
  dstT.topLeftCorner<3,3>() = q.toRotationMatrix();

  dstT(0,3) = t[0];
  dstT(1,3) = t[1];
  dstT(2,3) = t[2];
  dstT(3,3) = 1.0;
}

void eigenmat_to_raw( const Matrix4d& T, double * quat, double * t)
{
  assert( T(3,3) == 1 );
  Quaterniond q( T.topLeftCorner<3,3>() );
  quat[0] = q.w();
  quat[1] = q.x();
  quat[2] = q.y();
  quat[3] = q.z();
  t[0] = T(0,3);
  t[1] = T(1,3);
  t[2] = T(2,3);
}

void rawyprt_to_eigenmat( const double * ypr, const double * t, Matrix4d& dstT )
{
  dstT = Matrix4d::Identity();
  Vector3d eigen_ypr;
  eigen_ypr << ypr[0], ypr[1], ypr[2];
  dstT.topLeftCorner<3,3>() = ypr2R( eigen_ypr );
  dstT(0,3) = t[0];
  dstT(1,3) = t[1];
  dstT(2,3) = t[2];
}

Matrix4d rawyprt_to_eigenmat( double yy, double pp, double rr, double tx, double ty, double tz )
{
    Matrix4d dstT = Matrix4d::Identity();
    Vector3d eigen_ypr;
    eigen_ypr << yy, pp, rr;
    dstT.topLeftCorner<3,3>() = ypr2R( eigen_ypr );
    dstT(0,3) = tx;
    dstT(1,3) = ty;
    dstT(2,3) = tz;
    return dstT;
}

void eigenmat_to_rawyprt( const Matrix4d& T, double * ypr, double * t)
{
  assert( T(3,3) == 1 );
  Vector3d T_cap_ypr = R2ypr( T.topLeftCorner<3,3>() );
  ypr[0] = T_cap_ypr(0);
  ypr[1] = T_cap_ypr(1);
  ypr[2] = T_cap_ypr(2);

  t[0] = T(0,3);
  t[1] = T(1,3);
  t[2] = T(2,3);
}


////////////////////////////////////////////////////////////////////////////////
