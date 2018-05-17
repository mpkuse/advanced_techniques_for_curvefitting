#pragma once

#include <iostream>
#include <fstream>

#include <random>

#include <Eigen/Core>

using namespace std;
using namespace Eigen;

#include <ceres/ceres.h>


const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

template <typename Derived>
void printEigenMatrix(const string& filename, const MatrixBase<Derived>& a)
{
  std::ofstream file(filename);
  if( file.is_open() )
  {
    file << a.format(CSVFormat) << endl;
    cout << "\033[1;32m" <<"Written to file: "<< filename  << "\033[0m\n";
  }
  else
  {
    cout << "\033[1;31m" << "FAIL TO OPEN FILE for writing: "<< filename << "\033[0m\n";

  }
}


void printMatrix2d( const string& filename, const double * D, int nRows, int nCols )
{
  std::ofstream file(filename);

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
}

void printMatrix1d( const string& filename, const double * D, int n  )
{
  std::ofstream file(filename);
  file << D[0];
  for( int i=1 ; i<n ; i++ )
    file << ", " << D[i] ;
  file << "\n";
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
