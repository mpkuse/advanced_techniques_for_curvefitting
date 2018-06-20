// Input a 3d point cloud and corresponding 2d points. Objective is to get
// pose of the camera.


#include "utilities.h"



int main()
{
    //
    // Create Data. ie. 3d-2d pairs
    MatrixXd p_X;

    MatrixXd M;
    load_pointcloud( "../data3d/mannequin_mini.dat", 4, M );
    int npts = M.rows();
    p_X = MatrixXd( 4, npts );
    p_X << M.leftCols(3).transpose(), MatrixXd::Ones(1,npts);

    MatrixXd c_u, c_X;
    Matrix4d c_T_p;
    c_T_p = rawyprt_to_eigenmat( 20., 30., 40., -50., -320, -50 );
    c_X = c_T_p * p_X;
    c_u = MatrixXd::Ones( 3, npts );
    c_u.row(0).array() = c_X.row(0).array() / c_X.row(2).array(); //perspective divide
    c_u.row(1).array() = c_X.row(1).array() / c_X.row(2).array(); //perspective divide


    printEigenMatrix( "p_X.txt", p_X );
    printEigenMatrix( "c_X.txt", c_X );
    printEigenMatrix( "c_u.txt", c_u );
    printMatrixInfo( "p_X.txt", p_X );
    printMatrixInfo( "c_X.txt", c_X );
    printMatrixInfo( "c_u.txt", c_u );

}
